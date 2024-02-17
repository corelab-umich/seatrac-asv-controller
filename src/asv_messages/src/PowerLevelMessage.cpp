#include "asv_messages/PowerLevelMessage.h"

#include <asv_utils/utils/utils.h>
#include <stdexcept>

using namespace asv::utils;

namespace asv::messages
{

const size_t PowerLevelMessage::buffer_size;
const unsigned char PowerLevelMessage::sink_id;

PowerLevelMessage PowerLevelMessage::decode(const unsigned char* buf, size_t buf_len)
{
  if (buf_len != PowerLevelMessage::buffer_size || buf[0] != 0 || buf[1] != 0xff || buf[6] != sink_id)
  {
    throw std::invalid_argument("failed to decode");
  }

  PowerLevelMessage out{};
  out.timestamp = DatetimeMessage::decode(buf + 7, 8);
  out.pack_current = 0.002 * to_short(buf + 33);
  out.load_current = 0.002 * to_short(buf + 35);
  out.pack_voltage = 0.001 * to_ushort(buf + 37);
  out.soc_percent = 0.002 * to_ushort(buf + 39);

  return out;
}

std::vector<unsigned char> PowerLevelMessage::encode()
{
  std::vector<unsigned char> out{};
  out.reserve(PowerLevelMessage::buffer_size);

  // sync byte 1
  out.push_back(0x00);
  // sync byte 2
  out.push_back(0xFF);
  // (length) in little endian
  out.push_back(0x33);
  out.push_back(0x00);
  // (relay)
  out.push_back(0xFF);
  // message type (status reply)
  out.push_back(0x08);
  // sink id
  out.push_back(PowerLevelMessage::sink_id);
  // timestamp
  add_to_end(out, timestamp.encode());
  // filler bytes
  add_to_end(out, std::vector<unsigned char>(18, 0));
  // add contents of struct
  add_to_end(out, short_to_bytes(pack_current * 200));
  add_to_end(out, short_to_bytes(load_current * 200));
  add_to_end(out, ushort_to_bytes(pack_voltage * 100));
  add_to_end(out, ushort_to_bytes(soc_percent * 200));
  // filler bytes
  add_to_end(out, std::vector<unsigned char>(4, 0));
  // checksum bytes
  add_checksum_bytes(out, PowerLevelMessage::buffer_size - 6 - 2);

  return out;
}
}  // namespace asv::messages