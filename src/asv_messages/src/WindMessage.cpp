#include "asv_messages/WindMessage.h"

#include <asv_utils/utils/utils.h>
#include <stdexcept>

using namespace asv::utils;

namespace asv::messages
{

const size_t WindMessage::buffer_size;
const unsigned char WindMessage::sink_id;

WindMessage WindMessage::decode(const unsigned char* buf, size_t buf_len)
{
  if (buf_len != WindMessage::buffer_size || buf[0] != 0 || buf[1] != 0xff || buf[6] != sink_id)
  {
    throw std::invalid_argument("failed to decode");
  }

  WindMessage out{};

  out.timestamp = DatetimeMessage::decode(buf + 7, 8);
  out.apparent_speed = to_ushort(buf + 15) * 0.002;
  out.apparent_angle = to_short(buf + 17) * 0.01;
  out.temperature = to_short(buf + 19) * 0.01;
  out.pressure = to_short(buf + 21) * 0.01;

  return out;
}

std::vector<unsigned char> WindMessage::encode()
{
  std::vector<unsigned char> out{};
  out.reserve(WindMessage::buffer_size);

  // sync byte 1
  out.push_back(0x00);
  // sync byte 2
  out.push_back(0xFF);
  // (length) in little endian
  out.push_back(0x11);
  out.push_back(0x00);
  // (relay)
  out.push_back(0xFF);
  // message type (status reply)
  out.push_back(0x08);
  // sink id
  out.push_back(WindMessage::sink_id);
  // timestamp
  add_to_end(out, timestamp.encode());
  add_to_end(out, ushort_to_bytes(apparent_speed * 200));
  add_to_end(out, short_to_bytes(apparent_angle * 100));
  add_to_end(out, short_to_bytes(temperature * 100));
  add_to_end(out, short_to_bytes(pressure * 100));
  // checksum bytes
  add_checksum_bytes(out, WindMessage::buffer_size - 6 - 2);

  return out;
}
}  // namespace asv::messages