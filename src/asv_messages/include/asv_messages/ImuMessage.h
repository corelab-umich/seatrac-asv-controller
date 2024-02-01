#pragma once

#include <cstddef>
#include "asv_messages/DatetimeMessage.h"

namespace asv::messages
{
    /**
     * C/C++ style struct derived from the IMU udp packet structure published by the SeaTrac asv.
     * Provides methods for encoding into bytes buffer, decoding from bytes buffer, and converting into ros message(s).
     */
    struct ImuMessage
    {
        unsigned short sink_id;
        DatetimeMessage timestamp;

        // units for RPY: ??? (guessing degrees)
        float roll;
        float min_roll;
        float max_roll;
        float pitch;
        float min_pitch;
        float max_pitch;
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

        static ImuMessage decode(const unsigned char *buf, size_t buf_len);
    };
}