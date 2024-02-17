std_msgs/Header header
# current roll in degrees
float32 roll
# minimum roll in degrees
float32 min_rollfloat32
# maximum roll in degrees
float32 max_roll
# current pitch in degrees
float32 pitch
# minimum pitch in degrees
float32 min_pitch
# maximum pitch in degrees
float32 max_pitch
# current heading in degrees (always positive)
float32 heading
# minimum heading in degrees (always positive)
float32 min_heading
# maximum heading in degrees (always positive)
float32 max_heading

# current roll gyro rate in deg/sec
float32 roll_gyro_rate
# current pitch gyro rate in deg/sec
float32 pitch_gyro_rate
# current heading gyro rate in deg/sec
float32 heading_gyro_rate

# current linear acceleration along x (forwards) in m/s^2
float32 acceleration_x
# current linear acceleration along y (starboard) in m/s^2
float32 acceleration_y
# current linear acceleration along z (down) in m/s^2
float32 acceleration_z
# maximum linear acceleration along x (forwards) in m/s^2
float32 max_acceleration_x
# maximum linear acceleration along y (starboard) in m/s^2
float32 max_acceleration_y
# maximum linear acceleration along z (down) in m/s^2
float32 max_acceleration_z

# z (heave) in meters
float32 z
# minumum z (heave) in meters
float32 min_z
# maximum z (heave) in meters
float32 max_z