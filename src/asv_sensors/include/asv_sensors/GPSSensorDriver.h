#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <messages/msg/asv_gps.hpp>

namespace asv::ros
{
/**
 * Subscribes to raw packet data published by the SensorPacketPublisher node
 * and decodes relevant GPS packets to publish to gps topic.
 */
class GPSSensorDriver : public rclcpp::Node
{
public:
  GPSSensorDriver();

  /**
   * @brief Callback function for subscriber that's subscribed to raw package
   * data. If the packet data is an encoded GPS message, it parses the packet
   * data and publishes an messages::AsvGps message as well as an
   * sensor_msgs::msg::NavSatFix message
   * @param buf raw packet data
   */
  void callback(const std_msgs::msg::ByteMultiArray::SharedPtr raw_data);

private:
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr packet_sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_msg_pub_;
  rclcpp::Publisher<::messages::msg::AsvGps>::SharedPtr asv_gps_msg_pub_;
};
}  // namespace asv::ros
