#include "asv_utils/utils/utils.h"

namespace asv::utils
{
    unsigned short to_ushort(const unsigned char *buf, bool little_endian)
    {
        unsigned short out = 0;
        if (little_endian)
        {
            out = buf[1];
            out <<= 8;
            out |= buf[0];
        }
        else
        {
            out = buf[0];
            out <<= 8;
            out |= buf[1];
        }

        return out;
    }

    short to_short(const unsigned char *buf, bool little_endian)
    {
        unsigned short temp = to_ushort(buf, little_endian);

        return (short)temp;
    }
}