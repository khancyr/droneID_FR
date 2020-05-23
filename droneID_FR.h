/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
/*
 * Initial version by Pierre Kancir on May 2020 on funding by Airbot Systems
 *
 */


#pragma once
#include <cstdint>
#include <chrono>

/**
  * Cette class implemente le système d'identification numérique des drones français conformément à
  * This class implement the french drone identification frame as stated in
  * https://www.legifrance.gouv.fr/eli/arrete/2019/12/27/ECOI1934044A/jo/texte
  */
class droneIDFR {
public:
    /**
     * Constructeur de la librairie. Utilise une valeur par default pour l'ID du drone
     */
    droneIDFR(): _droneID("ILLEGAL_DRONE_APPELEZ_POLICE17") {};
    /**
     *
     * Taille maximale de la frame
     *
     */
    static constexpr uint8_t FRAME_PAYLOAD_LEN_MAX = 251;

    /**
     * Setter pour les coordonnées GPS en entier en centidegrees
     * @param lat
     * @param lon
     */
    void set_lat_lon(int32_t lat, int32_t lon) {
        _old_latitude = _latitude;
        _old_longitude = _longitude;
        _latitude = lat * 1e-2;
        _longitude = lon * 1e-2;
        _travelled_distance = distanceBetween(_latitude, _longitude, _old_latitude, _old_longitude);
    }
    /**
     * Setter pour les coordonnées GPS en double en centidegrees
     * Converti les valeurs en entiers.
     * @param lat
     * @param lon
     */
    void set_lat_lon(double lat, double lon) {
        _old_latitude = _latitude;
        _old_longitude = _longitude;
        _latitude = lat * 1e5;
        _longitude = lon * 1e5;
        _travelled_distance = distanceBetween(_latitude, _longitude, _old_latitude, _old_longitude);
    }
    /**
     * Setter pour l'altitude en MSL (Mean Sea Level)/ Niveau au dessus de la mer en m
     * @param alt
     */
    void set_altitude(int16_t alt) {
        _altitude = alt;
    }
    /**
     * Setter pour la hauteur au sol par rapport au point de décollage en m
     * @param height
     */
    void set_heigth(int16_t height) {
        _height = height;
    }
    /**
     * Setter pour les coordonnées GPS en entiers en centidegrees
     * @param lat
     * @param lon
     */
    void set_home_lat_lon(int32_t lat, int32_t lon) {
        _home_latitude = lat * 1e-2;
        _home_longitude = lon * 1e-2;
    }
    /**
     * Setter pour les coordonnées GPS en entiers en centidegrees
     * @param lat
     * @param lon
     */
    void set_home_lat_lon(double lat, double lon) {
        _home_latitude = lat * 1e5;
        _home_longitude = lon * 1e5;
    }
    /**
     * Setter pour la vitesse au sol en m/s
     * @param ground_speed
     */
    void set_ground_speed(uint8_t ground_speed) {
        _ground_speed = ground_speed;
    }
    /**
     * Cap du drone en degree par rapport au nord
     * @param heading
     */
    void set_heading(uint16_t heading) {
        _heading = heading;
    }

    /**
     * Setter pour l'id du drone.
     * Utiliser cette fonction pour changer l'id par défault
     * @param id_value
     */
    void set_drone_id(const char* id_value) {
        // don't use std::copy as it isn't support on all targets like espressif32 sdk !
        memcpy(_droneID, id_value, TLV_LENGTH[ID_FR]);
    }

    /**
     * Genère la frame 802.11 beacon complète
     * @param full_frame beacon frame buffer
     * @param start_from starting offset on the buffer
     * @return buffer space used
     */
    uint8_t generate_beacon_frame(uint8_t* full_frame, uint16_t start_from)
    {
        // Vendor specific 802.11 beacon frame
        full_frame[start_from] = FRAME_VS;
        start_from++;
        const uint16_t payload_marker = start_from;
        start_from++;

        for (auto i = 0; i<3; i++) {
            full_frame[start_from] = FRAME_OUI[i];
            start_from++;
        }
/*        full_frame[start_from] = FRAME_VS_TYPE;
        start_from++;*/
        const uint8_t payload_size = generate_drone_frame(full_frame, start_from);  // remove payload
        full_frame[payload_marker] = payload_size + 3;  // +OUI
        return start_from + payload_size;
    }
    /**
     * Genère le contenu de l'identification drone
     * @param full_frame beacon frame buffer
     * @param start_from starting offset on the buffer
     * @return buffer space used
     */
    uint8_t generate_drone_frame(uint8_t* full_frame, uint16_t start_from) {
        uint8_t count = 0;
        // Protocol version
        full_frame[start_from + count] = PROTOCOL_VERSION;
        count++;
        full_frame[start_from + count] = TLV_LENGTH[PROTOCOL_VERSION];
        count++;
        full_frame[start_from + count] = FRAME_VS_TYPE;
        count++;
        // Drone ID FR
        full_frame[start_from + count] = ID_FR;
        count++;
        full_frame[start_from + count] = TLV_LENGTH[ID_FR];
        count++;
        for (auto i = 0; i < TLV_LENGTH[ID_FR]; i++) {
            full_frame[start_from + count] = _droneID[i];
            count++;
        }
        // LATITUDE
        full_frame[start_from + count] = LATITUDE;
        count++;
        full_frame[start_from + count] = TLV_LENGTH[LATITUDE];
        count++;
        for (auto i = TLV_LENGTH[LATITUDE] - 1; i >= 0; i--) {
            full_frame[start_from + count] = (get_2_complement(_latitude) >> (8 * i)) & 0xFF;
            count++;
        }
        // LONGITUDE
        full_frame[start_from + count] = LONGITUDE;
        count++;
        full_frame[start_from + count] = TLV_LENGTH[LONGITUDE];
        count++;
        for (auto i = TLV_LENGTH[LONGITUDE] - 1; i >= 0; i--) {
            full_frame[start_from + count] = (get_2_complement(_longitude) >> (8 * i)) & 0xFF;
            count++;
        }
        // ALTITUDE
        full_frame[start_from + count] = ALTITUDE;
        count++;
        full_frame[start_from + count] = TLV_LENGTH[ALTITUDE];
        count++;
        for (auto i = TLV_LENGTH[ALTITUDE] - 1; i >= 0; i--) {
            full_frame[start_from + count] = (get_2_complement(_altitude) >> (8 * i)) & 0xFF;
            count++;
        }
        // HEIGHT
        full_frame[start_from + count] = HEIGTH;
        count++;
        full_frame[start_from + count] = TLV_LENGTH[HEIGTH];
        count++;
        for (auto i = TLV_LENGTH[HEIGTH] - 1; i >= 0; i--) {
            full_frame[start_from + count] = (get_2_complement(_height) >> (8 * i)) & 0xFF;
            count++;
        }
        // HOME LATITUDE
        full_frame[start_from + count] = HOME_LATITUDE;
        count++;
        full_frame[start_from + count] = TLV_LENGTH[HOME_LATITUDE];
        count++;
        for (auto i = TLV_LENGTH[HOME_LATITUDE] - 1; i >= 0; i--) {
            full_frame[start_from + count] = (get_2_complement(_home_latitude) >> (8 * i)) & 0xFF;
            count++;
        }
        // HOME LONGITUDE
        full_frame[start_from + count] = HOME_LONGITUDE;
        count++;
        full_frame[start_from + count] = TLV_LENGTH[HOME_LONGITUDE];
        count++;
        for (auto i = TLV_LENGTH[HOME_LONGITUDE] - 1; i >= 0; i--) {
            full_frame[start_from + count] = (get_2_complement(_home_longitude) >> (8 * i)) & 0xFF;
            count++;
        }
        // GROUND SPEED
        full_frame[start_from + count] = GROUND_SPEED;
        count++;
        full_frame[start_from + count] = TLV_LENGTH[GROUND_SPEED];
        count++;
        full_frame[start_from + count] = _ground_speed;
        count++;
        // HEADING
        full_frame[start_from + count] = HEADING;
        count++;
        full_frame[start_from + count] = TLV_LENGTH[HEADING];
        count++;
        for (auto i = TLV_LENGTH[HEADING] - 1; i >= 0; i--) {
            full_frame[start_from + count] = (get_2_complement(_heading) >> (8 * i)) & 0xFF;
            count++;
        }
        return count;
        // TODO: check lenght
    }

    /**
     * Sauvegarde la dernière fois qu'une trame a été envoyé pour respecter le timing d'une trame toutes les 3s
     */
    void set_last_send() {
        _last_send = std::chrono::high_resolution_clock::now();
        _travelled_distance = 0;
    }

    /**
     * Notifie quand 3s sont passés pour envoyer une nouvelle trame.
     * @return true if elapse time is > 3s
     */
    bool has_pass_time() const {
        std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - _last_send;
        return elapsed.count() >= FRAME_TIME_LIMIT;
    }
    /**
     * Notifie quand le drone a bougé de plus de 30m en moins de 3s.
     * @return true if distance travelled > 30m
     */
    bool has_pass_distance() const {
        return _travelled_distance >= FRAME_DISTANCE_LIMIT;
    }
    /**
     * Notifie si la condition de distance ou de temps est passé pour envoyer une nouvelle trame.
     * @return
     */
    bool time_to_send() const {
        return has_pass_time() || has_pass_distance();
    }

private:
    /**
     * Temps limite entre deux trames en s
     */
    static constexpr uint8_t FRAME_TIME_LIMIT = 3;  // in s
    /**
     * Distance limite entre deux trames en m
     */
    static constexpr uint8_t FRAME_DISTANCE_LIMIT = 30; // in m
    /**
     * Enumeration des types de données à envoyer
     */
    enum DATA_TYPE: uint8_t {
      RESERVED = 0,
      PROTOCOL_VERSION = 1,
      ID_FR = 2,
      ID_ANSI_CTA = 3,
      LATITUDE = 4,        // In WS84 in degree * 1e5
      LONGITUDE = 5,       // In WS84 in degree * 1e5
      ALTITUDE = 6,        // In MSL in m
      HEIGTH = 7,          // From Home in m
      HOME_LATITUDE = 8,   // In WS84 in degree * 1e5
      HOME_LONGITUDE = 9,  // In WS84 in degree * 1e5
      GROUND_SPEED = 10,   // In m/s
      HEADING = 11,        // Heading in degree from north 0 to 359.
      NOT_DEFINED_END = 12,
    };

    /**
     * Tableau TLV (TYPE, LENGTH, VALUE) avec les tailles attendu des différents type données.
     */
    static constexpr uint8_t TLV_LENGTH[] {
            0,  // [DATA_TYPE::RESERVED]
            1,  // [DATA_TYPE::PROTOCOL_VERSION]
            30, // [DATA_TYPE::ID_FR]
            0,  // [DATA_TYPE::ID_ANSI_CTA]
            4,  // [DATA_TYPE::LATITUDE]
            4,  // [DATA_TYPE::LONGITUDE]
            2,  // [DATA_TYPE::ALTITUDE]
            2,  // [DATA_TYPE::HEIGTH]
            4,  // [DATA_TYPE::HOME_LATITUDE]
            4,  // [DATA_TYPE::HOME_LONGITUDE]
            1,  // [DATA_TYPE::GROUND_SPEED]
            2,  // [DATA_TYPE::HEADING]
    };

    static constexpr uint8_t FRAME_COPTER_ID = 3;
    static constexpr uint8_t FRAME_PLANE_ID = 4;

    /**
     * Beacon frame VS:
     */
    static constexpr uint8_t FRAME_VS = 0XDD;
    /**
     * Beacon frame OUI
     */
    const uint8_t FRAME_OUI[3] = {0x6A, 0x5C, 0x35};
    /**
     * Beacon frame VS TYPE
     */
    static constexpr uint8_t FRAME_VS_TYPE = 1;

    int32_t _latitude;
    int32_t _longitude;
    int16_t _altitude;
    int16_t _height;
    int32_t _home_latitude;
    int32_t _home_longitude;
    uint8_t _ground_speed;
    uint16_t _heading;
    uint8_t _droneID[TLV_LENGTH[ID_FR]+1]; // +1 for null termination
    std::chrono::system_clock::time_point _last_send = std::chrono::system_clock::now();
    // for travelled distance calculation
    int32_t _old_latitude;
    int32_t _old_longitude;
    int32_t _travelled_distance;


    static inline uint32_t get_2_complement(int32_t value) {
        return value & 0xFFFFFFFF;
    }
    static inline uint16_t get_2_complement(int16_t value) {
        return value & 0xFFFF;
    }

    // Taken from TinyGPS++
    /**
     * Calcule une approximation de la distance entre deux coordonnées WS84 (GPS)
     * @param lat1
     * @param long1
     * @param lat2
     * @param long2
     * @return distance en m
     */
    static int32_t distanceBetween(double lat1, double long1, double lat2, double long2)
    {
        // returns distance in meters between two positions, both specified
        // as signed decimal-degrees latitude and longitude. Uses great-circle
        // distance computation for hypothetical sphere of radius 6372795 meters.
        // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
        // Courtesy of Maarten Lamers
        double delta = radians(long1-long2);
        double sdlong = sin(delta);
        double cdlong = cos(delta);
        lat1 = radians(lat1);
        lat2 = radians(lat2);
        double slat1 = sin(lat1);
        double clat1 = cos(lat1);
        double slat2 = sin(lat2);
        double clat2 = cos(lat2);
        delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
        delta = sq(delta);
        delta += sq(clat2 * sdlong);
        delta = sqrt(delta);
        double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
        delta = atan2(delta, denom);
        return static_cast<int32_t>(delta * 6372795);
    }
};

