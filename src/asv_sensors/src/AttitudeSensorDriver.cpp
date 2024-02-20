#include "asv_sensors/AttitudeSensorDriver.h"

#include <asv_messages/AttitudeMessage.h>
#include <std_msgs/msg/header.hpp>

#define SINK_ID_BYTE_INDEX 6

namespace asv::ros
{
AttitudeSensorDriver::AttitudeSensorDriver() : Node("attitude_sensor_driver")
{
  // use SensorDataQoS which which emphasized timeliness over reliability.
  packet_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
      "/raw_sensor_packets", rclcpp::SensorDataQoS(),
      std::bind(&AttitudeSensorDriver::callback, this, std::placeholders::_1));

  attitude_msg_pub_ = this->create_publisher<::messages::msg::Attitude>("/attitude", rclcpp::SensorDataQoS());
}

void AttitudeSensorDriver::callback(const std_msgs::msg::ByteMultiArray::SharedPtr raw_data)
{
  // check if the packet received is actually an attitude message packet
  if (raw_data->data.size() != asv::messages::AttitudeMessage::buffer_size ||
      raw_data->data[SINK_ID_BYTE_INDEX] != asv::messages::AttitudeMessage::sink_id)
  {
    std::cout << "size: " << raw_data->data.size() << ", sink id: " << (int)raw_data->data[SINK_ID_BYTE_INDEX]
              << std::endl;
    return;
  }

  auto asv_attitude_msg = asv::messages::AttitudeMessage::decode(raw_data->data.data(), raw_data->data.size());
  auto ros_attitude_msg = ::messages::msg::Attitude{};

  ros_attitude_msg.header.stamp = this->now();

  ros_attitude_msg.pitch = asv_attitude_msg.pitch;
  ros_attitude_msg.min_pitch = asv_attitude_msg.min_pitch;
  ros_attitude_msg.max_pitch = asv_attitude_msg.max_pitch;

  ros_attitude_msg.roll = asv_attitude_msg.roll;
  ros_attitude_msg.min_roll = asv_attitude_msg.min_roll;
  ros_attitude_msg.max_roll = asv_attitude_msg.max_roll;

  ros_attitude_msg.heading = asv_attitude_msg.heading;
  ros_attitude_msg.min_heading = asv_attitude_msg.min_heading;
  ros_attitude_msg.max_heading = asv_attitude_msg.max_heading;

  attitude_msg_pub_->publish(ros_attitude_msg);
}

}  // namespace asv::ros

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto node = std::make_shared<asv::ros::AttitudeSensorDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}