#include "asv_messages/AttitudeMessage.h"

#include <asv_utils/utils/utils.h>
#include <stdexcept>

using namespace asv::utils;

namespace asv::messages
{

const size_t AttitudeMessage::buffer_size;
const unsigned char AttitudeMessage::sink_id;

AttitudeMessage AttitudeMessage::decode(const unsigned char* buf, size_t buf_len)
{
  if (buf_len != AttitudeMessage::buffer_size || buf[0] != 0 || buf[1] != 0xff || buf[6] != sink_id)
  {
    throw std::invalid_argument("failed to decode");
  }
  AttitudeMessage out{};
  out.timestamp = DatetimeMessage::decode(buf + 7, 8);
  out.heading = 0.01 * to_ushort(buf + 15);
  out.pitch = 0.01 * to_short(buf + 17);
  out.min_pitch = 0.01 * to_short(buf + 19);
  out.max_pitch = 0.01 * to_short(buf + 21);
  out.roll = 0.01 * to_short(buf + 23);
  out.min_roll = 0.01 * to_short(buf + 25);
  out.max_roll = 0.01 * to_short(buf + 27);
  out.min_heading = 0.01 * to_ushort(buf + 29);
  out.max_heading = 0.01 * to_ushort(buf + 31);

  return out;
}

std::vector<unsigned char> AttitudeMessage::encode()
{
  std::vector<unsigned char> out{};
  out.reserve(AttitudeMessage::buffer_size);

  // sync byte 1
  out.push_back(0x00);
  // sync byte 2
  out.push_back(0xFF);
  // (length) in little endian
  out.push_back(0x1B);
  out.push_back(0x00);
  // (relay)
  out.push_back(0xFF);
  // message type (status reply)
  out.push_back(0x08);
  // sink id
  out.push_back(AttitudeMessage::sink_id);
  // timestamp
  add_to_end(out, timestamp.encode());
  add_to_end(out, ushort_to_bytes(heading * 100));
  add_to_end(out, short_to_bytes(pitch * 100));
  add_to_end(out, short_to_bytes(min_pitch * 100));
  add_to_end(out, short_to_bytes(max_pitch * 100));
  add_to_end(out, short_to_bytes(roll * 100));
  add_to_end(out, short_to_bytes(min_roll * 100));
  add_to_end(out, short_to_bytes(max_roll * 100));
  add_to_end(out, ushort_to_bytes(min_heading * 100));
  add_to_end(out, ushort_to_bytes(max_heading * 100));

  add_checksum_bytes(out, AttitudeMessage::buffer_size - 6 - 2);

  return out;
}
}  // namespace asv::messages