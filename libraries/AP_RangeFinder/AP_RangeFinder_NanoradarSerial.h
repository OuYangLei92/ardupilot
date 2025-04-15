#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_NANORADAR_SERIAL_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_NanoradarSerial : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_NanoradarSerial(_state, _params);
    }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

private:

    AP_RangeFinder_NanoradarSerial(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    // Protocol parse status
    typedef enum {
        NANORADAR_PROTOCOL_PARSE_HEADER1 = 0,
        NANORADAR_PROTOCOL_PARSE_HEADER2,
        NANORADAR_PROTOCOL_PARSE_MSG_ID1,
        NANORADAR_PROTOCOL_PARSE_MSG_ID2,
        NANORADAR_PROTOCOL_PARSE_MSG_DATA,
        NANORADAR_PROTOCOL_PARSE_END1,
        NANORADAR_PROTOCOL_PARSE_END2
    }NANORADAR_PROTOCOL_PARSE_STATUS;
    
    // Object Message: NAR24、NAR15 Protocol e.g.
    typedef struct PACKED {
        uint8_t index;
        uint8_t rcs;             // res = rcs * 0.5 - 50
        uint8_t distance_h8;     // dist = distance_h8 * 256 + distance_l8 (cm)
        uint8_t distance_l8;
        uint8_t res1;
        uint8_t vel_h : 3;       // vel = vel_h * 256 + vel_l (m/s)
        uint8_t res2  : 3;
        uint8_t roll_count : 2;
        uint8_t vel_l;
        uint8_t snr;             // receive snr = snr - 127
    }object_message_t;

    // Object Message: Extended Object Message, UAM231、UAM221 Protocol e.g.
    typedef struct PACKED {
        uint8_t index : 4;
        uint8_t distance_h4 : 4; // dist = (distance_h4 << 16) | (distance_m8 << 8) | distance_l8 (cm)
        uint8_t snr;             // receive snr = snr * 0.5
        uint8_t distance_m8;
        uint8_t distance_l8;
        uint8_t radar_mode : 4;
        uint8_t confidence : 4;  // distance confidence = confidence / 10
        uint8_t vel_h : 3;       // vel = (vel_h *256 + vel_l)*0.1-100 (m/s)
        uint8_t res1 : 3;         
        uint8_t roll_count : 2; 
        uint8_t vel_l;            
        uint8_t crc;
    }object_message_extend_t;

    // Protocol frame struct
    typedef struct PACKED{
        uint16_t header;
        uint16_t msg_id;
        union PACKED{
            uint8_t buf[8];
            object_message_t object_msg;
            object_message_extend_t object_msg_extend;
        }payload;
        uint16_t end;
    }nanoradar_frame_t;

    void update(void) override;

    // get a reading
    bool get_reading(float &reading_m) override;

    uint16_t read_timeout_ms() const override { return 300; }

    bool parse_byte(uint8_t c);

    uint8_t check_sum(const uint8_t* data, uint8_t len);

    uint8_t _data_index;
    uint8_t _extend_version_cnt;
    uint32_t _last_heartbeat_time_ms;

    NANORADAR_PROTOCOL_PARSE_STATUS _parse_status;
    nanoradar_frame_t _rx_frame;
};

#endif  // AP_RANGEFINDER_NANORADAR_SERIAL_ENABLED
