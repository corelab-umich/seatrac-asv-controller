#pragma once

#include <cstddef>
#include <vector>

#include "asv_messages/DatetimeMessage.h"

namespace asv::messages
{
/**
 * C++ style struct derived from the Attitude udp packet structure published by the
 * SeaTrac asv. Provides methods for encoding into bytes buffer, decoding from
 * bytes buffer, and converting into ros message(s).
 */
struct AttitudeMessage
{
  // timestamp for this message
  DatetimeMessage timestamp;

  // current pitch in degrees
  float pitch;
  // minimum pitch in degrees
  float min_pitch;
  // maximum pitch in degrees
  float max_pitch;
  // current roll in degrees
  float roll;
  // minimum roll in degrees
  float min_roll;
  // maximum roll in degrees
  float max_roll;
  // current heading in degrees (always positive)
  float heading;
  // minimum heading in degrees (always positive)
  float min_heading;
  // maximum heading in degrees (always positive)
  float max_heading;

  // size of data section of udp packet (no udp header info) send by boat (inlcudes checksum and header bytes)
  static const size_t buffer_size{ 35 };
  // sink id for message
  static const unsigned char sink_id{ 76 };

  /**
   * @brief Parses the provided byte buffer according to the SeaTrac Attitude udp
   * packet specification and returns a populated `AttitudeMessage` struct
   * @param buf byte buffer containing the raw packet data
   * @param buf_len size of the byte buffer. Must be the same size as
   * `buffer_size`
   * @return populated `AttitudeMessage` struct
   */
  static AttitudeMessage decode(const unsigned char* buf, size_t buf_len);

  /**
   * @brief Encodes this `AttitudeMessage` struct into a byte buffer according to the
   * SeaTrac Attitude udp packet specification. No error checking is performed.
   */
  std::vector<unsigned char> encode();
};
}  // namespace asv::messages