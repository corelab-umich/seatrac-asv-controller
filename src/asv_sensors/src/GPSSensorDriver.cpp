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

  nav_sat_msg_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/nav_sat_fix", rclcpp::SensorDataQoS());
  asv_gps_msg_pub_ = this->create_publisher<::messages::msg::AsvGps>("/gps", rclcpp::SensorDataQoS());
}

void GPSSensorDriver::callback(const std_msgs::msg::ByteMultiArray::SharedPtr raw_data)
{
  // check if the packet received is actually an gps message packet
  if (raw_data->data.size() != asv::messages::GPSMessage::buffer_size ||
      raw_data->data[SINK_ID_BYTE_INDEX] != asv::messages::GPSMessage::sink_id)
  {
    return;
  }

  RCLCPP_DEBUG_STREAM(this->get_logger(), "Received new GPS message with sink id: " << (int) asv::messages::GPSMessage::sink_id);

  auto asv_gps_msg = asv::messages::GPSMessage::decode(raw_data->data.data(), raw_data->data.size());
  auto nav_sat_msg = sensor_msgs::msg::NavSatFix{};

  nav_sat_msg.header.stamp = this->now();

  nav_sat_msg.latitude = to_degrees(asv_gps_msg.latitude);
  nav_sat_msg.longitude = to_degrees(asv_gps_msg.longitude);

  nav_sat_msg_pub_->publish(nav_sat_msg);

  auto asv_gps_ros_msg = ::messages::msg::AsvGps{};
  asv_gps_ros_msg.header.stamp = nav_sat_msg.header.stamp;
  asv_gps_ros_msg.latitude = asv_gps_msg.latitude;
  asv_gps_ros_msg.longitude = asv_gps_msg.longitude;
  asv_gps_ros_msg.kts = asv_gps_msg.kts;
  asv_gps_ros_msg.heading = asv_gps_msg.heading;
  asv_gps_ros_msg.current_kts = asv_gps_msg.current_kts;
  asv_gps_ros_msg.current_heading = asv_gps_msg.current_heading;
  asv_gps_ros_msg.wind_kts = asv_gps_msg.wind_kts;
  asv_gps_ros_msg.wind_heading = asv_gps_msg.wind_heading;

  asv_gps_msg_pub_->publish(asv_gps_ros_msg);
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