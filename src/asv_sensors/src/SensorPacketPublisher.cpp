#include "asv_sensors/SensorPacketPublisher.h"

#include <std_msgs/msg/multi_array_dimension.hpp>
#include <asv_utils/networking/UDPClient.h>
#include <iostream>

#include <climits> 

#define IPADDRESS "127.0.0.2"
#define UDP_PORT 62002
#define BUFFER_SIZE 1024

namespace asv::ros
{
SensorPacketPublisher::SensorPacketPublisher() : Node("sensor_packet_publisher")
{
  // use SensorDataQoS which which emphasized timeliness over reliability.
  packet_pub_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("/raw_sensor_packets", rclcpp::SensorDataQoS());
}

void SensorPacketPublisher::handle_message(const unsigned char* buf, size_t buf_size)
{
  std::cout << "Received packet # " << ++count_ << " with size of " << buf_size << " bytes.\n";
  std_msgs::msg::ByteMultiArray msg{};
  msg.data = std::vector<unsigned char>{ buf, buf + buf_size };
  // leave msg.layout empty since this isn't a multidimensional array.

  packet_pub_->publish(std::move(msg));
}
}  // namespace asv::ros

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto spp = std::make_shared<asv::ros::SensorPacketPublisher>();
  spp->declare_parameter("ip_address", IPADDRESS);
  spp->declare_parameter("udp_port", UDP_PORT);
  unsigned short port_num;
  int port_num_int = spp->get_parameter("udp_port").as_int();
  if (port_num_int < 0 || port_num_int > USHRT_MAX) {
    throw new std::invalid_argument("invalid port num! must be within range of short int.");
  } else {
    port_num = (unsigned short) port_num_int;
  }
  asv::networking::UDPClient client{ spp->get_parameter("ip_address").as_string(),
                                     port_num, BUFFER_SIZE };
  // start receiving packets
  std::thread r([&] { client.start(*spp); });
  // keep the node from shutting down
  rclcpp::spin(spp);
  rclcpp::shutdown();
  return 0;
}