#pragma once

#include <asv_utils/networking/MessageReceiver.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

namespace asv::ros {
/**
 * Node to read UDP packets from the SeaTrac boat and publish raw packet data to
 * a topic. Other nodes, such as the IMUDriver node, subscribe to this topic and
 * process the relevant packets.
 *
 * Inherits from MessageReceiver such that this node's handle_message function
 * can be used with UDPClient
 */
class SensorPacketPublisher : public rclcpp::Node,
                              public networking::MessageReceiver {
  public:
    SensorPacketPublisher();

    /**
     * @brief This function is called whenever a new udp packet is received.
     * It published the packet data to a topic for downstream nodes.
     * @param buf raw packet data
     * @param buf_size size of buf
     */
    void handle_message(const unsigned char *buf, size_t buf_size) override;

  private:
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr packet_pub_;
    size_t count_;
};
} // namespace asv::ros