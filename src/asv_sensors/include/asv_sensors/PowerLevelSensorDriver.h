#pragma once

#include <rclcpp/rclcpp.hpp>
#include <messages/msg/power_level.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

namespace asv::ros
{
/**
 * Subscribes to raw packet data published by the SensorPacketPublisher node
 * and decodes relevant Power Level packets to publish to power_level topic.
 */
class PowerLevelSensorDriver : public rclcpp::Node
{
public:
  PowerLevelSensorDriver();

  /**
   * @brief Callback function for subscriber that's subscribed to raw package
   * data. If the packet data is an encoded power level message, it parses the packet
   * data and publishes an messages::PowerLevel message
   * @param buf raw packet data
   */
  void callback(const std_msgs::msg::ByteMultiArray::SharedPtr raw_data);

private:
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr packet_sub_;
  rclcpp::Publisher<::messages::msg::PowerLevel>::SharedPtr power_level_msg_pub_;
};
}  // namespace asv::ros
