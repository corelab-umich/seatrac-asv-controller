#pragma once

#include <cstddef>
#include <vector>

#include "asv_messages/DatetimeMessage.h"

namespace asv::messages
{
/**
 * C++ style struct derived from the IMU udp packet structure published by the
 * SeaTrac asv. Provides methods for encoding into bytes buffer, decoding from
 * bytes buffer, and converting into ros message(s).
 */
struct ImuMessage
{
  // timestamp for this message
  DatetimeMessage timestamp;

  // current roll in degrees
  float roll;
  // minimum roll in degrees
  float min_roll;
  // maximum roll in degrees
  float max_roll;
  // current pitch in degrees
  float pitch;
  // minimum pitch in degrees
  float min_pitch;
  // maximum pitch in degrees
  float max_pitch;
  // current heading in degrees (always positive)
  float heading;
  // minimum heading in degrees (always positive)
  float min_heading;
  // maximum heading in degrees (always positive)
  float max_heading;

  // current roll gyro rate in deg/sec
  float roll_gyro_rate;
  // current pitch gyro rate in deg/sec
  float pitch_gyro_rate;
  // current heading gyro rate in deg/sec
  float heading_gyro_rate;

  // current linear acceleration along x (forwards) in m/s^2
  float acceleration_x;
  // current linear acceleration along y (starboard) in m/s^2
  float acceleration_y;
  // current linear acceleration along z (down) in m/s^2
  float acceleration_z;
  // maximum linear acceleration along x (forwards) in m/s^2
  float max_acceleration_x;
  // maximum linear acceleration along y (starboard) in m/s^2
  float max_acceleration_y;
  // maximum linear acceleration along z (down) in m/s^2
  float max_acceleration_z;

  // z (heave) in meters
  float z;
  // minumum z (heave) in meters
  float min_z;
  // maximum z (heave) in meters
  float max_z;

  // size of data section of udp packet (no udp header info) send by boat (inlcudes checksum and header bytes)
  static const size_t buffer_size{ 59 };
  // sink id for message
  static const unsigned char sink_id{ 86 };

  /**
   * @brief Parses the provided byte buffer according to the SeaTrac IMU udp
   * packet specification and returns a populated `ImuMessage` struct
   * @param buf byte buffer containing the raw packet data
   * @param buf_len size of the byte buffer. Must be the same size as
   * `buffer_size`
   * @return populated `ImuMessage` struct
   */
  static ImuMessage decode(const unsigned char* buf, size_t buf_len);

  /**
   * @brief Encodes this `ImuMessage` struct into a byte buffer according to the
   * SeaTrac IMU udp packet specification. No error checking is performed.
   */
  std::vector<unsigned char> encode();
};
}  // namespace asv::messages