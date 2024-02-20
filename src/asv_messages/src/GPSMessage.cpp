#include "asv_messages/GPSMessage.h"

#include <asv_utils/utils/utils.h>
#include <stdexcept>

using namespace asv::utils;

namespace asv::messages
{

const size_t GPSMessage::buffer_size;
const unsigned char GPSMessage::sink_id;

GPSMessage GPSMessage::decode(const unsigned char* buf, size_t buf_len)
{
  if (buf_len != GPSMessage::buffer_size || buf[0] != 0 || buf[1] != 0xff || buf[6] != sink_id)
  {
    throw std::invalid_argument("failed to decode");
  }

  GPSMessage out{};
  out.timestamp = DatetimeMessage::decode(buf + 7, 8);
  out.lattitude = to_double(buf + 15);
  out.longitude = to_double(buf + 23);
  out.knots = to_ushort(buf + 31) * 0.002;
  out.heading = to_ushort(buf + 33) * 0.01;
  out.current_knots = to_ushort(buf + 35) * 0.002;
  out.current_heading = to_ushort(buf + 37) * 0.01;
  out.wind_knots = to_ushort(buf + 37) * 0.002;
  out.wind_heading = to_ushort(buf + 39) * 0.01;

  return out;
}

// TODO: finish this @aspratap
std::vector<unsigned char> GPSMessage::encode()
{
  std::vector<unsigned char> out{};
  out.reserve(GPSMessage::buffer_size);

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
  out.push_back(GPSMessage::sink_id);
  // timestamp
  add_to_end(out, timestamp.encode());
  // filler bytes
  // add_to_end(out, std::vector<unsigned char>(18, 0));
  // // add contents of struct
  // add_to_end(out, short_to_bytes(pack_current * 200));
  // add_to_end(out, short_to_bytes(load_current * 200));
  // add_to_end(out, ushort_to_bytes(pack_voltage * 100));
  // add_to_end(out, ushort_to_bytes(soc_percent * 200));
  // // filler bytes
  // add_to_end(out, std::vector<unsigned char>(4, 0));
  // checksum bytes
  add_checksum_bytes(out, GPSMessage::buffer_size - 6 - 2);

  return out;
}
}  // namespace asv::messages