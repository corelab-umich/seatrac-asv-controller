#pragma once

#include <rclcpp/rclcpp.hpp>
#include <messages/msg/wind.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

namespace asv::ros
{
/**
 * Subscribes to raw packet data published by the SensorPacketPublisher node
 * and decodes relevant wind sensor packets to publish to wind topic.
 */
class WindSensorDriver : public rclcpp::Node
{
public:
  WindSensorDriver();

  /**
   * @brief Callback function for subscriber that's subscribed to raw package
   * data. If the packet data is an encoded wind sensor message, it parses the packet
   * data and publishes an messages::Wind message
   * @param buf raw packet data
   */
  void callback(const std_msgs::msg::ByteMultiArray::SharedPtr raw_data);

private:
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr packet_sub_;
  rclcpp::Publisher<::messages::msg::Wind>::SharedPtr wind_msg_pub_;
};
}  // namespace asv::ros
