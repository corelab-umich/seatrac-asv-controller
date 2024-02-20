#include "asv_sensors/GPSSensorDriver.h"

#include <asv_messages/GPSMessage.h>
#include <std_msgs/msg/header.hpp>
#include <asv_utils/utils/utils.h>

#define SINK_ID_BYTE_INDEX 6

using namespace asv::utils;

namespace asv::ros
{
GPSSensorDriver::GPSSensorDriver() : Node("gps_sensor_driver")
{
  // use SensorDataQoS which which emphasized timeliness over reliability.
  packet_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
      "/raw_sensor_packets", rclcpp::SensorDataQoS(),
      std::bind(&GPSSensorDriver::callback, this, std::placeholders::_1));

  gps_msg_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps", rclcpp::SensorDataQoS());
}

void GPSSensorDriver::callback(const std_msgs::msg::ByteMultiArray::SharedPtr raw_data)
{
  // check if the packet received is actually an gps message packet
  if (raw_data->data.size() != asv::messages::GPSMessage::buffer_size ||
      raw_data->data[SINK_ID_BYTE_INDEX] != asv::messages::GPSMessage::sink_id)
  {
    std::cout << "size: " << raw_data->data.size() << ", sink id: " << (int)raw_data->data[SINK_ID_BYTE_INDEX]
              << std::endl;
    return;
  }

  auto asv_gps_msg = asv::messages::GPSMessage::decode(raw_data->data.data(), raw_data->data.size());
  auto ros_gps_msg = sensor_msgs::msg::NavSatFix{};

  ros_gps_msg.header.stamp = this->now();

  ros_gps_msg.latitude = to_degrees(asv_gps_msg.lattitude);
  ros_gps_msg.latitude = to_degrees(asv_gps_msg.lattitude);

  gps_msg_pub_->publish(ros_gps_msg);
}

}  // namespace asv::ros

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto node = std::make_shared<asv::ros::GPSSensorDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}