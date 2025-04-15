/*
* Copyright (C) 2016  Intel Corporation. All rights reserved.
*
* This file is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the
* Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This file is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AP_RangeFinder_NanoradarSerial.h"

#if AP_RANGEFINDER_NANORADAR_SERIAL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>

#define NANORADAR_PROTOCOL_HEADER 0xAA    // Frame Header
#define NANORADAR_PROTOCOL_END    0x55    // Frame End

#define NANORADAR_STATUS_MSG_ID 0x0201
#define NANORADAR_OBJECT_MSG_ID 0x070C
#define NANORADAR_PROTOCOL_PAYLOAD_LEN 8

extern const AP_HAL::HAL& hal;

AP_RangeFinder_NanoradarSerial::AP_RangeFinder_NanoradarSerial(
    RangeFinder::RangeFinder_State &_state,
    AP_RangeFinder_Params &_params):
    AP_RangeFinder_Backend_Serial(_state, _params)
{
    _extend_version_cnt = 100;
    _parse_status = NANORADAR_PROTOCOL_PARSE_HEADER1;
}

// update the state of the sensor
void AP_RangeFinder_NanoradarSerial::update(void)
{
    if (get_reading(state.distance_m)) {
        state.signal_quality_pct = get_signal_quality_pct();
        // update range_valid state based on distance measured
        state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > read_timeout_ms()) {
        if (AP_HAL::millis() - _last_heartbeat_time_ms > read_timeout_ms()) {
            // no heartbeat, must be disconnected
            set_status(RangeFinder::Status::NotConnected);
        } else {
            // Have heartbeat, just no data. Probably because this sensor doesn't output data when there is no relative motion infront of the radar.
            // This case has special pre-arm check handling
            set_status(RangeFinder::Status::NoData);
        }
    }
}

// read - return last value measured by sensor
bool AP_RangeFinder_NanoradarSerial::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    uint16_t count = 0;
    uint32_t distance_cm = 0;
    uint32_t sum_cm = 0;

    for (auto i=0; i<8192; i++) {
        uint8_t b = 0;

        if (!uart->read(b)) {
            break;
        }

        if (parse_byte(b)) {
            switch (_rx_frame.msg_id & 0x0F8F)
            {
                case NANORADAR_OBJECT_MSG_ID:
                {
                    if (_extend_version_cnt >= 200 && _rx_frame.payload.buf[7] == check_sum(_rx_frame.payload.buf, 7)) {
                        const object_message_extend_t* p_obj_msg = &_rx_frame.payload.object_msg_extend;
                        distance_cm = ((uint32_t)(p_obj_msg->distance_h4) << 16) | ((uint32_t)(p_obj_msg->distance_m8) << 8)  | p_obj_msg->distance_l8;
                        
                    }
                    else if (_extend_version_cnt == 0) {
                        const object_message_t* p_obj_msg = &_rx_frame.payload.object_msg;
                        distance_cm = ((uint32_t)(p_obj_msg->distance_h8) << 8) | p_obj_msg->distance_l8;
                    }
                    else {
                        if (_rx_frame.payload.buf[7] == check_sum(_rx_frame.payload.buf, 7)) {
                            _extend_version_cnt += 1;
                        } else {
                            if (_extend_version_cnt > 0 ) {
                                _extend_version_cnt -= 1;
                            }
                        }
                        break;
                    }

                    sum_cm += distance_cm;
                    ++count;

                    break;
                }
                case NANORADAR_STATUS_MSG_ID:
                    _last_heartbeat_time_ms = AP_HAL::millis();
                    break;
                default:
                    break;
            }
        }
    }

    if (count > 0) {
        reading_m = sum_cm / (count * 100.0f);
        if (reading_m > 655.0f) {
            reading_m = 655.35f;
        }
        return true;
    }

    return false;
}

uint8_t AP_RangeFinder_NanoradarSerial::check_sum(const uint8_t* data, uint8_t len)
{
    uint16_t sum = 0;
    while (len--) {
        sum += *data;
        ++data;
    }
    return (uint8_t)(sum & 0xFF);
}

bool AP_RangeFinder_NanoradarSerial::parse_byte(uint8_t c)
{
    switch (_parse_status) {
        case NANORADAR_PROTOCOL_PARSE_HEADER1:
            if (NANORADAR_PROTOCOL_HEADER == c) {
                _parse_status = NANORADAR_PROTOCOL_PARSE_HEADER2;
            }
            break;
        
        case NANORADAR_PROTOCOL_PARSE_HEADER2:
            if (NANORADAR_PROTOCOL_HEADER == c) {
                _rx_frame.header = NANORADAR_PROTOCOL_HEADER | (NANORADAR_PROTOCOL_HEADER << 8);
                _parse_status = NANORADAR_PROTOCOL_PARSE_MSG_ID1;
            } else {
                _parse_status = NANORADAR_PROTOCOL_PARSE_HEADER1;
            }
            break;
        
        case NANORADAR_PROTOCOL_PARSE_MSG_ID1:
            _rx_frame.msg_id = c;
            _parse_status = NANORADAR_PROTOCOL_PARSE_MSG_ID2;
            break;
        
        case NANORADAR_PROTOCOL_PARSE_MSG_ID2:
            _rx_frame.msg_id |= ((uint16_t)c << 8);
            _data_index= 0;
            _parse_status = NANORADAR_PROTOCOL_PARSE_MSG_DATA;
            break;
        
        case NANORADAR_PROTOCOL_PARSE_MSG_DATA:
            _rx_frame.payload.buf[_data_index++] = c;
            if (_data_index>= NANORADAR_PROTOCOL_PAYLOAD_LEN) {
                _parse_status = NANORADAR_PROTOCOL_PARSE_END1;
            }
            break;
        
        case NANORADAR_PROTOCOL_PARSE_END1:
            _rx_frame.end = c;
            _parse_status = NANORADAR_PROTOCOL_PARSE_END2;
            break;
        
        case NANORADAR_PROTOCOL_PARSE_END2:
            _rx_frame.end |= (uint16_t)c << 8;
            _parse_status = NANORADAR_PROTOCOL_PARSE_HEADER1;
            if (_rx_frame.end == (NANORADAR_PROTOCOL_END | (NANORADAR_PROTOCOL_END << 8))) {
                return true;
            }
            break;
        
        default:
            _parse_status = NANORADAR_PROTOCOL_PARSE_HEADER1;
            break;
    }
    return false;
}

#endif  // AP_RANGEFINDER_NANORADAR_SERIAL_ENABLED
