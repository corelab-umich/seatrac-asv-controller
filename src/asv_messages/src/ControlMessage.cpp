#include "asv_messages/ControlMessage.h"

#include <asv_utils/utils/utils.h>
#include <stdexcept>

using namespace asv::utils;

namespace asv::messages
{

const size_t ControlMessage::buffer_size;
const unsigned char ControlMessage::board_id;
const unsigned char ControlMessage::sink_id;
const unsigned char ControlMessage::function;

// ControlMessage ControlMessage::decode(const unsigned char* buf, size_t buf_len)
// {
//   if (buf_len != ControlMessage::buffer_size || buf[0] != 0 || buf[1] != 0xff || buf[6] != sink_id)
//   {
//     throw std::invalid_argument("failed to decode");
//   }

//   ControlMessage out{};

//   out.timestamp = DatetimeMessage::decode(buf + 7, 8);
//   out.apparent_speed = to_ushort(buf + 15) * 0.002;
//   out.apparent_angle = to_short(buf + 17) * 0.01;
//   out.temperature = to_short(buf + 19) * 0.01;
//   out.pressure = to_short(buf + 21) * 0.01;

//   return out;
// }

std::vector<unsigned char> ControlMessage::encode()
{
  std::vector<unsigned char> out{};
  out.reserve(ControlMessage::buffer_size);

  // sync byte 1
  out.push_back(0x00);
  // sync byte 2
  out.push_back(0xFF);
  // (length) in little endian
  out.push_back(0x0A);
  out.push_back(0x00);
  // (relay)
  out.push_back(0x08);
  // message type (command)
  out.push_back(0x0B);
  // board id (8)
  out.push_back(ControlMessage::board_id);
  // sink id
  out.push_back(ControlMessage::sink_id);
  // function
  out.push_back(ControlMessage::function);
  // heading command
  add_to_end(out, float_to_bytes(desired_heading));
  // rpm command
  add_to_end(out, float_to_bytes(desired_rpm));
  // checksum bytes
  add_checksum_bytes(out, ControlMessage::buffer_size - 6 - 2);

  return out;
}
}  // namespace asv::messages