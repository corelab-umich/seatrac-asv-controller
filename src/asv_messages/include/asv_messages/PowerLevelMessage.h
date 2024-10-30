#pragma once

#include <cstddef>
#include <vector>

#include "asv_messages/DatetimeMessage.h"

namespace asv::messages
{
/**
 * C++ style struct derived from the Power Level udp packet structure published by the
 * SeaTrac asv. Provides methods for encoding into bytes buffer, decoding from
 * bytes buffer, and converting into ros message(s).
 */
struct PowerLevelMessage
{
  // current timestamp for message
  DatetimeMessage timestamp;
  // battery pack current in amps. defined as (load current – charger current): negative pack current means the chargers
  // are charging more than the load is drawing, so the pack is being charged
  float pack_current;
  // load current in amps
  float load_current;
  // battery pack voltage in volts (always positive)
  float pack_voltage;
  // SOC percent
  float soc_percent;

  // size of data section of udp packet (no udp header info) send by boat (inlcudes checksum and header bytes)
  static const size_t buffer_size{ 47 };
  // sink id for message
  static const unsigned char sink_id{ 64 };

  /**
   * @brief Parses the provided byte buffer according to the SeaTrac Power Level udp
   * packet specification and returns a populated `PowerLevel` struct
   * @param buf byte buffer containing the raw packet data
   * @param buf_len size of the byte buffer. Must be the same size as
   * `buffer_size`
   * @return populated `PowerLevel` message struct
   */
  static PowerLevelMessage decode(const unsigned char* buf, size_t buf_len);

  /**
   * @brief Encodes this `PowerLevel` struct into a byte buffer according to the
   * SeaTrac Power Level udp packet specification. No error checking is performed.
   */
  std::vector<unsigned char> encode();
};
}  // namespace asv::messages