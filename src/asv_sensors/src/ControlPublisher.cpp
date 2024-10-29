// system headers
#include <chrono>
#include <cmath>
#include <random>

// other headers
#include <rclcpp/rclcpp.hpp>
#include <asv_utils/networking/UDPSender.h>

// ASV Message Types
#include <asv_messages/ControlMessage.h>

// Other Message Types
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <messages/msg/commands.hpp>

using namespace std::chrono_literals;
using namespace asv::networking;

#define LOCALHOST "127.0.0.2"
#define PORT 62002


class ControlPublisher : public rclcpp::Node
{
public:
  ControlPublisher() : Node("control_command_publisher"), udp_sender_()
  {

    control_sub_ = this->create_subscription<::messages::msg::Commands>("/asv_command", rclcpp::SensorDataQoS(), std::bind(&ControlPublisher::packet_publish, this, std::placeholders::_1));

    this->declare_parameter("ip_address", LOCALHOST);
    this->declare_parameter("udp_port", PORT);

    udp_sender_.init(this->get_parameter("ip_address").as_string(), this->get_parameter("udp_port").as_int());
  }

private:

    void packet_publish(const ::messages::msg::Commands ros_cmd_msg)
    {
        asv::messages::ControlMessage control_msg{};
        
        control_msg.desired_heading = ros_cmd_msg.heading;
        control_msg.desired_rpm = kts_to_rpm(ros_cmd_msg.speed_kts);

        udp_sender_.send_message(control_msg.encode());
    }

    float kts_to_rpm(float kts)
    {
        //TODO: Implement interpolation
        float rpm = kts * 0.0;
        return rpm;
    }

    rclcpp::Subscription<::messages::msg::Commands>::SharedPtr control_sub_;
    asv::networking::UDPSender udp_sender_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  rclcpp::spin(std::make_shared<ControlPublisher>());
  rclcpp::shutdown();
  return 0;
}