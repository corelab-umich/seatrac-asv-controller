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
struct ControlMessage
{
  // current timestamp for message
  DatetimeMessage timestamp;

  // Desired true heading (0-359 degrees)
  float desired_heading;
  // Desired motor RPM
  float desired_rpm;


  // size of data section of udp packet (no udp header info) send by boat (inlcudes checksum and header bytes)
  static const size_t buffer_size{ 19 };

  // board id for message
  static const unsigned char board_id{ 8 };
  // sink id for message
  static const unsigned char sink_id{ 82 };
  // function ID
  static const unsigned char function{ 2 };

  /**
   * @brief Parses the provided byte buffer according to the SeaTrac wind sensor udp
   * packet specification and returns a populated `WindSensor` struct
   * @param buf byte buffer containing the raw packet data
   * @param buf_len size of the byte buffer. Must be the same size as
   * `buffer_size`
   * @return populated `WindSensor` message struct
   */
  static ControlMessage decode(const unsigned char* buf, size_t buf_len);

  /**
   * @brief Encodes this `ControlMessage` struct into a byte buffer according to the
   * SeaTrac wind sensor udp packet specification. No error checking is performed.
   */
  std::vector<unsigned char> encode();
};
}  // namespace asv::messages