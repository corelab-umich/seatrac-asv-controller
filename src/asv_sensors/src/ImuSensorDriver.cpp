#include "asv_sensors/ImuSensorDriver.h"

#include <asv_messages/ImuMessage.h>
#include <asv_utils/utils/utils.h>
#include <math.h>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>

#define SINK_ID_BYTE_INDEX 6

using asv::utils::to_radians;

namespace asv::ros
{
ImuSensorDriver::ImuSensorDriver() : Node("imu_sensor_driver")
{
  // use SensorDataQoS which which emphasized timeliness over reliability.
  packet_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
      "/raw_sensor_packets", rclcpp::SensorDataQoS(),
      std::bind(&ImuSensorDriver::callback, this, std::placeholders::_1));

  imu_msg_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS());
}

void ImuSensorDriver::callback(const std_msgs::msg::ByteMultiArray::SharedPtr raw_data)
{
  // check if the packet received is actually an IMU message packet
  if (raw_data->data.size() != asv::messages::ImuMessage::buffer_size ||
      raw_data->data[SINK_ID_BYTE_INDEX] != asv::messages::ImuMessage::sink_id)
  {
    std::cout << "size: " << raw_data->data.size() << ", sink id: " << (int)raw_data->data[SINK_ID_BYTE_INDEX]
              << std::endl;
    return;
  }

  auto asv_imu_msg = asv::messages::ImuMessage::decode(raw_data->data.data(), raw_data->data.size());
  auto ros_imu_msg = sensor_msgs::msg::Imu{};
  ros_imu_msg.header.stamp = this->now();

  // convert rpy to quaternion
  tf2::Quaternion quat{};
  // converts about fixed axes
  quat.setRPY(to_radians(asv_imu_msg.roll), to_radians(asv_imu_msg.pitch), to_radians(asv_imu_msg.heading));
  quat.normalize();
  ros_imu_msg.orientation.w = quat.getW();
  ros_imu_msg.orientation.x = quat.getX();
  ros_imu_msg.orientation.y = quat.getY();
  ros_imu_msg.orientation.z = quat.getZ();

  // set angular velocities
  ros_imu_msg.angular_velocity.x = to_radians(asv_imu_msg.roll_gyro_rate);
  ros_imu_msg.angular_velocity.y = to_radians(asv_imu_msg.pitch_gyro_rate);
  ros_imu_msg.angular_velocity.z = to_radians(asv_imu_msg.heading_gyro_rate);

  // set linear accelerations
  ros_imu_msg.linear_acceleration.x = asv_imu_msg.acceleration_x;
  ros_imu_msg.linear_acceleration.y = asv_imu_msg.acceleration_y;
  ros_imu_msg.linear_acceleration.z = asv_imu_msg.acceleration_z;

  imu_msg_pub_->publish(ros_imu_msg);
}

}  // namespace asv::ros

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto node = std::make_shared<asv::ros::ImuSensorDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}