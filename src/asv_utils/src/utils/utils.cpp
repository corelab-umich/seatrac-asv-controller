#include "asv_utils/utils/utils.h"

#include <math.h>
#include <stdexcept>

namespace asv::utils {

unsigned short to_ushort(const unsigned char *buf, bool little_endian) {
    unsigned short out = 0;
    if (little_endian) {
        out = buf[1];
        out <<= 8;
        out |= buf[0];
    } else {
        out = buf[0];
        out <<= 8;
        out |= buf[1];
    }

    return out;
}

short to_short(const unsigned char *buf, bool little_endian) {
    unsigned short temp = to_ushort(buf, little_endian);

    return (short)temp;
}

float to_radians(float degrees) { return degrees * M_PI / 180; }

float to_degrees(float radians) { return radians / M_PI * 180; }

std::vector<unsigned char> ushort_to_bytes(unsigned short num,
                                           bool little_endian) {
    std::vector<unsigned char> out{};
    out.reserve(2);
    if (little_endian) {
        out.push_back((num) & 0xFF);
        out.push_back((num >> 8) & 0xFF);
    } else {
        out.push_back((num >> 8) & 0xFF);
        out.push_back((num) & 0xFF);
    }

    return out;
}

std::vector<unsigned char> short_to_bytes(short num, bool little_endian) {
    return ushort_to_bytes((unsigned short)num, little_endian);
}

void add_checksum_bytes(std::vector<unsigned char> &buf,
                        int message_specific_data_length) {
    unsigned char *pData = buf.data();
    int iDataLength = message_specific_data_length;
    int iHeaderLength = 6;
    int iSum1 = 0, iSum2 = 0;
    int iLength = iHeaderLength + iDataLength;
    while (iLength--) {
        iSum1 += *pData++;
        if (iSum1 >= 255)
            iSum1 -= 255;
        iSum2 += iSum1;
        if (iSum2 >= 255)
            iSum2 -= 255;
    }
    if (!((iSum1 <= 255) && (iSum2 <= 255))) {
        throw std::invalid_argument("checksum failed");
    }
    int iCheck1 = 255 - (iSum1 + iSum2) % 255;
    int iCheck2 = 255 - (iSum1 + iCheck1) % 255;
    buf.push_back((unsigned char)iCheck1);
    buf.push_back((unsigned char)iCheck2);
}

} // namespace asv::utils