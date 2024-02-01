#pragma once

#include <cstddef>

namespace asv::messages
{
    /**
     * C/C++ style struct derived from the Datetime udp packet structure published by the SeaTrac asv.
     * Provides methods for encoding into bytes buffer, decoding from bytes buffer, and converting into ros message(s).
     */
    struct DatetimeMessage
    {
        // Year
        unsigned short year;
        // Month: Jan = 1, Dec = 12
        unsigned short month;
        // Day: 1 to 31
        unsigned short day;
        // Hour: 0 to 23
        unsigned short hour;
        // Minute: 0 to 59
        unsigned short minute;
        // Second: 0 to 59
        unsigned short second;
        // Hundredths: 0 to 99
        unsigned short hundreths;

        static DatetimeMessage decode(const unsigned char *buf, size_t buf_len);
    };
}