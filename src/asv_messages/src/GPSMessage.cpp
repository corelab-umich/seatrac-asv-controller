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
  out.latitude = to_double(buf + 15);
  out.longitude = to_double(buf + 23);
  out.kts = to_ushort(buf + 31) * 0.002;
  out.heading = to_ushort(buf + 33) * 0.01;
  out.current_kts = to_ushort(buf + 35) * 0.002;
  out.current_heading = to_ushort(buf + 37) * 0.01;
  out.wind_kts = to_ushort(buf + 37) * 0.002;
  out.wind_heading = to_ushort(buf + 39) * 0.01;

  return out;
}

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

  add_to_end(out, double_to_bytes(latitude));
  add_to_end(out, double_to_bytes(longitude));
  add_to_end(out, ushort_to_bytes(kts * 2000));
  add_to_end(out, ushort_to_bytes(heading * 100));
  add_to_end(out, ushort_to_bytes(current_kts * 2000));
  add_to_end(out, ushort_to_bytes(current_heading * 100));
  add_to_end(out, ushort_to_bytes(wind_kts * 2000));
  add_to_end(out, ushort_to_bytes(wind_heading * 100));

  // checksum bytes
  add_checksum_bytes(out, GPSMessage::buffer_size - 6 - 2);

  return out;
}
}  // namespace asv::messages