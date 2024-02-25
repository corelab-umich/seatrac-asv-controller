#pragma once

#include <vector>

/**
 * collection of various inline utility methods
 */

namespace asv::utils
{
/**
 * @brief reads the first 2 elements of the specified char buffer and translates
 * them into an unsigned short
 * @param buf pointer to a unsigned char buffer containing data to translate.
 * First 2 elements are read.
 * @param little_endian endian-ness of the data in the buf. true by default
 * @return parsed unsigned short
 */
unsigned short to_ushort(const unsigned char* buf, bool little_endian = true);

/**
 * @brief reads the first 2 elements of the specified char buffer and translates
 * them into a short
 * @param buf pointer to a unsigned char buffer containing data to translate.
 * First 2 elements are read.
 * @param little_endian endian-ness of the data in the buf. true by default
 * @return parsed short
 */
short to_short(const unsigned char* buf, bool little_endian = true);

/**
 * @brief reads the first 8 elements of the specified char buffer and translates
 * them into a double
 * @param buf pointer to a unsigned char buffer containing data to translate.
 * First 8 elements are read.
 * @param little_endian endian-ness of the data in the buf. true by default
 * @return parsed double
 */
double to_double(const unsigned char* buf, bool little_endian = true);

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
 * @brief converts radians to degrees
 * @param radians angle in radians
 * @return angle in degrees
 */
double to_degrees(double radians);

/**
 * @brief converts degrees to radians
 * @param degrees angle in degrees
 * @return angle in radians
 */
double to_radians(double degrees);


/**
 * @brief converts the provided short into into a byte buffer
 * @param num short to convert
 * @param little_endian endian-ness of the data to encode. true by default
 * @return byte buffer populated with provided short
 */
std::vector<unsigned char> short_to_bytes(short num, bool little_endian = true);

/**
 * @brief converts the provided unsigned short into into a byte buffer
 * @param num unsigned short to convert
 * @param little_endian endian-ness of the data to encode. true by default
 * @return byte buffer populated with provided unsigned short
 */
std::vector<unsigned char> ushort_to_bytes(unsigned short num, bool little_endian = true);

/**
 * @brief converts the provided double into into a byte buffer
 * @param num double to convert
 * @param little_endian endian-ness of the data to encode. true by default
 * @return byte buffer populated with provided double
 */
std::vector<unsigned char> double_to_bytes(double num, bool little_endian = true);

/**
 * @brief generates checksum bytes for when encoded message as per SeaTrac
 * specifications
 * @param buf source buffer for generating checksum. checksum bytes will be appended to end of this buffer as well.
 * (Buffer with message: starting with first byte (0x00))
 * @param message_specific_data_length length of message specific data (e.g. no header). see SeaTrac datasheet for more
 * information
 */
void add_checksum_bytes(std::vector<unsigned char>& buf, int message_specific_data_length);

/**
 * @brief simple method to add a vector to end of another vector
 * @param add_to_this contents of `add_from_this` will be appended to end of this vector
 * @param add_from_this contents of this vector (in order) will be appended to end of `add_to_this`
 */
template <typename T>
inline void add_to_end(std::vector<T>& add_to_this, const std::vector<T>& add_from_this)
{
  add_to_this.insert(add_to_this.end(), add_from_this.begin(), add_from_this.end());
}

}  // namespace asv::utils