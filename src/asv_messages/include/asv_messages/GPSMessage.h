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
  // lattitude in radians
  double lattitude;  
  // longitude in radians
  double longitude;
  // speed in knots towards target (always positive)
  float knots;
  // heading towards target (always positive)
  float heading;
  // current speed in knots (always positive)
  float current_knots;
  // heading (always positive)
  float current_heading;
  // wind speed in knots (always positive)
  float wind_knots;
  // wind heading (always positive)
  float wind_heading;

  // size of data section of udp packet (no udp header info) send by boat (inlcudes checksum and header bytes)
  static const size_t buffer_size{ 45 };
  // sink id for message
  static const unsigned char sink_id{ 77 };

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