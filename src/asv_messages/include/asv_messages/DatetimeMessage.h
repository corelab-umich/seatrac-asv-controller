#pragma once

#include <cstddef>
#include <vector>

namespace asv::messages {
/**
 * C/C++ style struct derived from the Datetime udp packet structure published
 * by the SeaTrac asv. Provides methods for encoding into bytes buffer, decoding
 * from bytes buffer, and converting into ros message(s).
 */
struct DatetimeMessage {
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

    static const size_t buffer_size{8};

    /**
     * @brief Parses the provided byte buffer according to the SeaTrac Datetime
     * udp packet specification and returns a populated DatetimeMessage struct
     * @param buf byte buffer containing the raw packet data
     * @param buf_len size of the byte buffer. Must be the same size as
     * `buffer_size`
     */
    static DatetimeMessage decode(const unsigned char *buf, size_t buf_len);

    /**
     * @brief Encodes this Datetime struct into a byte buffer according to the
     * SeaTrac Datetime udp packet specification
     */
    std::vector<unsigned char> encode();
};
} // namespace asv::messages