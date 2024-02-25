#pragma once

#include <cstddef>
#include <vector>

#include "asv_messages/DatetimeMessage.h"

namespace asv::messages
{
/**
 * C++ style struct derived from the GPS udp packet structure published by the
 * SeaTrac asv. Provides methods for encoding into bytes buffer, decoding from
 * bytes buffer, and converting into ros message(s).
 */
struct GPSMessage
{
  // current timestamp for message
  DatetimeMessage timestamp;
  // latitude in radians
  double latitude;  
  // longitude in radians
  double longitude;
  // speed in knots towards target (always positive)
  float kts;
  // heading towards target (always positive)
  float heading;
  // current speed in knots (always positive)
  float current_kts;
  // heading (always positive)
  float current_heading;
  // wind speed in knots (always positive)
  float wind_kts;
  // wind heading (always positive)
  float wind_heading;

  // size of data section of udp packet (no udp header info) send by boat (inlcudes checksum and header bytes)
  static const size_t buffer_size{ 45 };
  // sink id for message (80 is backup, 77 is regular)
  static const unsigned char sink_id{ 80 };

  /**
   * @brief Parses the provided byte buffer according to the SeaTrac GPS udp
   * packet specification and returns a populated `GPS` struct
   * @param buf byte buffer containing the raw packet data
   * @param buf_len size of the byte buffer. Must be the same size as
   * `buffer_size`
   * @return populated `GPS` message struct
   */
  static GPSMessage decode(const unsigned char* buf, size_t buf_len);

  /**
   * @brief Encodes this `GPS` struct into a byte buffer according to the
   * SeaTrac GPS udp packet specification. No error checking is performed.
   */
  std::vector<unsigned char> encode();
};
}  // namespace asv::messages