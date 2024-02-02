#pragma once

#include <cstddef>
#include <vector>

#include "asv_messages/DatetimeMessage.h"

namespace asv::messages {
/**
 * C/C++ style struct derived from the IMU udp packet structure published by the
 * SeaTrac asv. Provides methods for encoding into bytes buffer, decoding from
 * bytes buffer, and converting into ros message(s).
 */
struct ImuMessage {
    DatetimeMessage timestamp;

    // units for RPY: ??? (guessing degrees)
    float roll;
    float min_roll;
    float max_roll;
    float pitch;
    float min_pitch;
    float max_pitch;
    // heading must always be positive
    float heading;
    float min_heading;
    float max_heading;

    // gyro rates: deg/sec
    float roll_gyro_rate;
    float pitch_gyro_rate;
    float heading_gyro_rate;

    // accelerations: m/s^2
    // x: forwards
    // y: starboard
    // z: down
    float acceleration_x;
    float acceleration_y;
    float acceleration_z;
    float max_acceleration_x;
    float max_acceleration_y;
    float max_acceleration_z;

    // z (heave): meters
    float z;
    float min_z;
    float max_z;

    // data size of udp packet send by boat
    static const size_t buffer_size{59};
    // sink id for message
    static const unsigned char sink_id{86};

    /**
     * @brief Parses the provided byte buffer according to the SeaTrac IMU udp
     * packet specification and returns a populated ImuMessage struct
     * @param buf byte buffer containing the raw packet data
     * @param buf_len size of the byte buffer. Must be the same size as
     * `buffer_size`
     */
    static ImuMessage decode(const unsigned char *buf, size_t buf_len);

    /**
     * @brief Encodes this ImuMessage struct into a byte buffer according to the
     * SeaTrac IMU udp packet specification. No error checking is done.
     */
    std::vector<unsigned char> encode();
};
} // namespace asv::messages