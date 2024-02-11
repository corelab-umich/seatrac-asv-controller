#pragma once

#include <vector>

/**
 * collection of various inline utility methods
 */

namespace asv::utils {
/**
 * @brief reads the first 2 elements of the specified char buffer and translates
 * them into an unsigned short
 * @param buf pointer to a unsigned char buffer containing data to translate.
 * First 2 elements are read.
 * @param little_endian endian-ness of the data in the buf. true by default
 */
unsigned short to_ushort(const unsigned char *buf, bool little_endian = true);

/**
 * @brief reads the first 2 elements of the specified char buffer and translates
 * them into a short
 * @param buf pointer to a unsigned char buffer containing data to translate.
 * First 2 elements are read.
 * @param little_endian endian-ness of the data in the buf. true by default
 */
short to_short(const unsigned char *buf, bool little_endian = true);

/**
 * @brief converts degrees to radians
 * @param degrees angle in degrees
 * @return angle in radians
 */
float to_radians(float degrees);

/**
 * @brief converts radians to degrees
 * @param radians angle in radians
 * @return angle in degrees
 */
float to_degrees(float radians);

/**
 * @brief converts the provided short into into a byte buffer
 * @param num short to convert
 * @param little_endian endian-ness of the data to encode. true by default
 */
std::vector<unsigned char> short_to_bytes(short num, bool little_endian = true);

/**
 * @brief converts the provided unsigned short into into a byte buffer
 * @param num unsigned short to convert
 * @param little_endian endian-ness of the data to encode. true by default
 */
std::vector<unsigned char> ushort_to_bytes(unsigned short num,
                                           bool little_endian = true);

/**
 * @brief generates checksum bytes for when encoded message as per SeaTrac
 * specifications
 */
void add_checksum_bytes(std::vector<unsigned char> &buf,
                        int message_specific_data_length);

} // namespace asv::utils