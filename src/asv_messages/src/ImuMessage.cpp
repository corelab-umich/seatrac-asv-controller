#include "asv_messages/ImuMessage.h"

#include <asv_utils/utils/utils.h>
#include <stdexcept>

using namespace asv::utils;

namespace asv::messages
{

const size_t ImuMessage::buffer_size;
const unsigned char ImuMessage::sink_id;

ImuMessage ImuMessage::decode(const unsigned char* buf, size_t buf_len)
{
  if (buf_len != ImuMessage::buffer_size || buf[0] != 0 || buf[1] != 0xff || buf[6] != sink_id)
  {
    throw std::invalid_argument("failed to decode");
  }
  ImuMessage out{};
  out.timestamp = DatetimeMessage::decode(buf + 7, 8);
  out.roll = 0.01 * to_short(buf + 15);
  out.min_roll = 0.01 * to_short(buf + 17);
  out.max_roll = 0.01 * to_short(buf + 19);
  out.pitch = 0.01 * to_short(buf + 21);
  out.min_pitch = 0.01 * to_short(buf + 23);
  out.max_pitch = 0.01 * to_short(buf + 25);
  out.heading = 0.01 * to_ushort(buf + 27);
  out.roll_gyro_rate = 0.02 * to_short(buf + 29);
  out.pitch_gyro_rate = 0.02 * to_short(buf + 31);
  out.heading_gyro_rate = 0.02 * to_short(buf + 33);
  out.acceleration_x = 0.01 * to_short(buf + 35);
  out.acceleration_y = 0.01 * to_short(buf + 37);
  out.acceleration_z = 0.01 * to_short(buf + 39);
  out.max_acceleration_x = 0.01 * to_short(buf + 41);
  out.max_acceleration_y = 0.01 * to_short(buf + 43);
  out.max_acceleration_z = 0.01 * to_short(buf + 45);
  out.z = 0.001 * to_short(buf + 47);
  out.min_z = 0.001 * to_short(buf + 49);
  out.max_z = 0.001 * to_short(buf + 51);
  out.min_heading = 0.01 * to_ushort(buf + 53);
  out.max_heading = 0.01 * to_ushort(buf + 55);

  return out;
}

std::vector<unsigned char> ImuMessage::encode()
{
  std::vector<unsigned char> out{};
  out.reserve(ImuMessage::buffer_size);

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
  out.push_back(ImuMessage::sink_id);
  // timestamp
  add_to_end(out, timestamp.encode());
  // roll
  add_to_end(out, short_to_bytes(roll * 100));
  add_to_end(out, short_to_bytes(min_roll * 100));
  add_to_end(out, short_to_bytes(max_roll * 100));
  add_to_end(out, short_to_bytes(pitch * 100));
  add_to_end(out, short_to_bytes(min_pitch * 100));
  add_to_end(out, short_to_bytes(max_pitch * 100));
  // heading (assumes that heading is always positive)
  add_to_end(out, ushort_to_bytes(heading * 100));
  // angular velocities
  add_to_end(out, short_to_bytes(roll_gyro_rate * 200));
  add_to_end(out, short_to_bytes(pitch_gyro_rate * 200));
  add_to_end(out, short_to_bytes(heading_gyro_rate * 200));
  // linear accelerations
  add_to_end(out, short_to_bytes(acceleration_x * 100));
  add_to_end(out, short_to_bytes(acceleration_y * 100));
  add_to_end(out, short_to_bytes(acceleration_z * 100));
  add_to_end(out, short_to_bytes(max_acceleration_x * 100));
  add_to_end(out, short_to_bytes(max_acceleration_y * 100));
  add_to_end(out, short_to_bytes(max_acceleration_z * 100));
  // heave
  add_to_end(out, short_to_bytes(z * 1000));
  add_to_end(out, short_to_bytes(min_z * 1000));
  add_to_end(out, short_to_bytes(max_z * 1000));
  // min/max heading
  add_to_end(out, ushort_to_bytes(min_heading * 100));
  add_to_end(out, ushort_to_bytes(max_heading * 100));

  add_checksum_bytes(out, ImuMessage::buffer_size - 6 - 2);

  return out;
}
}  // namespace asv::messages