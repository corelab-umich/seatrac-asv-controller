#pragma once

#include <rclcpp/rclcpp.hpp>
#include <messages/msg/attitude.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

namespace asv::ros
{
/**
 * Subscribes to raw packet data published by the SensorPacketPublisher node
 * and decodes relevant Attitude sensor packets to publish to attitude topic.
 */
class AttitudeSensorDriver : public rclcpp::Node
{
public:
  AttitudeSensorDriver();

  /**
   * @brief Callback function for subscriber that's subscribed to raw package
   * data. If the packet data is an encoded attitude message, it parses the packet
   * data and publishes an messages::Attitude message
   * @param buf raw packet data
   */
  void callback(const std_msgs::msg::ByteMultiArray::SharedPtr raw_data);

private:
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr packet_sub_;
  rclcpp::Publisher<::messages::msg::Attitude>::SharedPtr attitude_msg_pub_;
};
}  // namespace asv::ros
