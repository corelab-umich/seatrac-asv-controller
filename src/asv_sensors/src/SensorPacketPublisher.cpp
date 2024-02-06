#include "asv_sensors/SensorPacketPublisher.h"

#include <std_msgs/msg/multi_array_dimension.hpp>
#include <asv_utils/networking/UDPClient.h>
#include <iostream>

#define IPADDRESS "127.0.0.1"
#define UDP_PORT 62001
#define BUFFER_SIZE 1024

namespace asv::ros
{
    SensorPacketPublisher::SensorPacketPublisher() : Node("sensor_packet_publisher")
    {
        // use SensorDataQoS which which emphasized timeliness over reliability.
        packet_pub_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("/raw_sensor_packets", rclcpp::SensorDataQoS());
    }

    void SensorPacketPublisher::handle_message(const unsigned char *buf, size_t buf_size)
    {
        std::cout << "Received packet # " << ++count_ << " with size of " << buf_size << " bytes.\n";
        std_msgs::msg::ByteMultiArray msg{};
        msg.data = std::vector<unsigned char>{buf, buf + buf_size};
        // leave msg.layout empty since this isn't a multidimensional array.

        packet_pub_->publish(std::move(msg));
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    asv::networking::UDPClient client{std::string(IPADDRESS), UDP_PORT, BUFFER_SIZE};
    auto spp = std::make_shared<asv::ros::SensorPacketPublisher>();
    // start receiving packets
    std::thread r([&]
                  { client.start(*spp); });
    // keep the node from shutting down
    rclcpp::spin(spp);
    rclcpp::shutdown();
    return 0;
}