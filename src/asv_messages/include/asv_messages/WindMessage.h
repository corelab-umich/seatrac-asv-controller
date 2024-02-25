#pragma once

#include <cstddef>
#include <vector>

#include "asv_messages/DatetimeMessage.h"

namespace asv::messages
{
/**
 * C++ style struct derived from the wind sensor udp packet structure published by the
 * SeaTrac asv. Provides methods for encoding into bytes buffer, decoding from
 * bytes buffer, and converting into ros message(s).
 */
struct WindMessage
{
  // current timestamp for message
  DatetimeMessage timestamp;

  // apparent speed measured in knots (always positive)
  float apparent_speed;
  // apparent angle measured in degrees
  float apparent_angle;
  // temperature in celsius
  float temperature;
  // pressure in bar
  float pressure;

  // size of data section of udp packet (no udp header info) send by boat (inlcudes checksum and header bytes)
  static const size_t buffer_size{ 25 };
  // sink id for message
  static const unsigned char sink_id{ 78 };

  /**
   * @brief Parses the provided byte buffer according to the SeaTrac wind sensor udp
   * packet specification and returns a populated `WindSensor` struct
   * @param buf byte buffer containing the raw packet data
   * @param buf_len size of the byte buffer. Must be the same size as
   * `buffer_size`
   * @return populated `WindSensor` message struct
   */
  static WindMessage decode(const unsigned char* buf, size_t buf_len);

  /**
   * @brief Encodes this `WindSensor` struct into a byte buffer according to the
   * SeaTrac wind sensor udp packet specification. No error checking is performed.
   */
  std::vector<unsigned char> encode();
};
}  // namespace asv::messages