#include "asv_messages/ImuMessage.h"

#include <asv_utils/utils/utils.h>
#include <stdexcept>

using asv::utils::to_short;
using asv::utils::to_ushort;

namespace asv::messages
{

    ImuMessage ImuMessage::decode(const unsigned char *buf, size_t buf_len)
    {
        if (buf_len != ImuMessage::buffer_size || buf[0] != 0 || buf[1] != 0xff)
        {
            throw std::invalid_argument("failed to decode");
        }
        ImuMessage out{};
        out.sink_id = buf[6];
        out.timestamp = DatetimeMessage::decode(buf + 7, 8);
        out.roll = 0.01 * to_short(buf + 15);
        out.min_roll = 0.01 * to_short(buf + 17);
        out.max_roll = 0.01 * to_short(buf + 19);
        out.pitch = 0.01 * to_short(buf + 21);
        out.min_pitch = 0.01 * to_short(buf + 23);
        out.max_pitch = 0.01 * to_short(buf + 25);
        out.heading = 0.01 * to_ushort(buf + 27);
        out.roll_gyro_rate = 0.02 * to_short(buf + 29);
        out.pitch_gyro_rate = 0.02 * to_short(buf + 31);
        out.heading_gyro_rate = 0.02 * to_short(buf + 33);
        out.acceleration_x = 0.01 * to_short(buf + 35);
        out.acceleration_y = 0.01 * to_short(buf + 37);
        out.acceleration_z = 0.01 * to_short(buf + 39);
        out.max_acceleration_x = 0.01 * to_short(buf + 41);
        out.max_acceleration_y = 0.01 * to_short(buf + 43);
        out.max_acceleration_z = 0.01 * to_short(buf + 45);
        out.z = 0.001 * to_short(buf + 47);
        out.min_z = 0.001 * to_short(buf + 49);
        out.max_z = 0.001 * to_short(buf + 51);
        out.min_heading = 0.01 * to_ushort(buf + 53);
        out.max_heading = 0.01 * to_ushort(buf + 55);

        return out;
    }
}