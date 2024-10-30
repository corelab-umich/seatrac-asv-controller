#include "asv_sensors/WindSensorDriver.h"

#include <asv_messages/WindMessage.h>
#include <std_msgs/msg/header.hpp>

#define SINK_ID_BYTE_INDEX 6

namespace asv::ros
{
WindSensorDriver::WindSensorDriver() : Node("wind_sensor_driver")
{
  // use SensorDataQoS which which emphasized timeliness over reliability.
  packet_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
      "/raw_sensor_packets", rclcpp::SensorDataQoS(),
      std::bind(&WindSensorDriver::callback, this, std::placeholders::_1));

  wind_msg_pub_ = this->create_publisher<::messages::msg::Wind>("/wind", rclcpp::SensorDataQoS());
}

void WindSensorDriver::callback(const std_msgs::msg::ByteMultiArray::SharedPtr raw_data)
{
  // check if the packet received is actually an wind message packet
  if (raw_data->data.size() != asv::messages::WindMessage::buffer_size ||
      raw_data->data[SINK_ID_BYTE_INDEX] != asv::messages::WindMessage::sink_id)
  {
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Received Wind sensor message");

  auto asv_wind_msg = asv::messages::WindMessage::decode(raw_data->data.data(), raw_data->data.size());
  auto ros_wind_msg = ::messages::msg::Wind{};

  ros_wind_msg.header.stamp = this->now();
  ros_wind_msg.apparent_speed = asv_wind_msg.apparent_speed;
  ros_wind_msg.apparent_angle = asv_wind_msg.apparent_angle;
  ros_wind_msg.temperature = asv_wind_msg.temperature;
  ros_wind_msg.pressure = asv_wind_msg.pressure;

  wind_msg_pub_->publish(ros_wind_msg);
}

}  // namespace asv::ros

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto node = std::make_shared<asv::ros::WindSensorDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}