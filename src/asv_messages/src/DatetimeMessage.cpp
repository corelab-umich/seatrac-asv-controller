#include "asv_messages/DatetimeMessage.h"

#include <asv_utils/utils/utils.h>
#include <stdexcept>
#include <iostream>

using asv::utils::to_ushort;

namespace asv::messages
{

    DatetimeMessage DatetimeMessage::decode(const unsigned char *buf, size_t buf_len)
    {
        if (buf_len != 8)
        {
            throw std::invalid_argument("incorrect buffer size. must be 8 bytes");
        }
        DatetimeMessage out{};
        out.year = to_ushort(buf);
        out.month = buf[2];
        out.day = buf[3];
        out.hour = buf[4];
        out.minute = buf[5];
        out.second = buf[6];
        out.hundreths = buf[7];

        std::cout << "year: " << out.year;
        return out;
    }
}