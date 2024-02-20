#include "asv_sensors/PowerLevelSensorDriver.h"

#include <asv_messages/PowerLevelMessage.h>
#include <std_msgs/msg/header.hpp>

#define SINK_ID_BYTE_INDEX 6

namespace asv::ros
{
PowerLevelSensorDriver::PowerLevelSensorDriver() : Node("power_level_sensor_driver")
{
  // use SensorDataQoS which which emphasized timeliness over reliability.
  packet_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
      "/raw_sensor_packets", rclcpp::SensorDataQoS(),
      std::bind(&PowerLevelSensorDriver::callback, this, std::placeholders::_1));

  power_level_msg_pub_ = this->create_publisher<::messages::msg::PowerLevel>("/power_level", rclcpp::SensorDataQoS());
}

void PowerLevelSensorDriver::callback(const std_msgs::msg::ByteMultiArray::SharedPtr raw_data)
{
  // check if the packet received is actually an power level message packet
  if (raw_data->data.size() != asv::messages::PowerLevelMessage::buffer_size ||
      raw_data->data[SINK_ID_BYTE_INDEX] != asv::messages::PowerLevelMessage::sink_id)
  {
    std::cout << "size: " << raw_data->data.size() << ", sink id: " << (int)raw_data->data[SINK_ID_BYTE_INDEX]
              << std::endl;
    return;
  }

  auto asv_power_level_msg = asv::messages::PowerLevelMessage::decode(raw_data->data.data(), raw_data->data.size());
  auto ros_power_level_msg = ::messages::msg::PowerLevel{};

  ros_power_level_msg.header.stamp = this->now();

  ros_power_level_msg.pack_current = asv_power_level_msg.pack_current;
  ros_power_level_msg.load_current = asv_power_level_msg.load_current;
  ros_power_level_msg.pack_voltage = asv_power_level_msg.pack_voltage;
  ros_power_level_msg.soc_percent = asv_power_level_msg.soc_percent;

  power_level_msg_pub_->publish(ros_power_level_msg);
}

}  // namespace asv::ros

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto node = std::make_shared<asv::ros::PowerLevelSensorDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}