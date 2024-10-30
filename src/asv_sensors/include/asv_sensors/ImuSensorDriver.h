#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

namespace asv::ros
{
/**
 * Subscribes to raw packet data published by the SensorPacketPublisher node
 * and decodes relevant IMU packets to publish to imu topic.
 */
class ImuSensorDriver : public rclcpp::Node
{
public:
  ImuSensorDriver();

  /**
   * @brief Callback function for subscriber that's subscribed to raw package
   * data. If the packet data is an encoded imu message, it parses the packet
   * data and publishes an sensor_msgs::Imu message
   * @param buf raw packet data
   */
  void callback(const std_msgs::msg::ByteMultiArray::SharedPtr raw_data);

private:
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr packet_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_msg_pub_;
};
}  // namespace asv::ros
