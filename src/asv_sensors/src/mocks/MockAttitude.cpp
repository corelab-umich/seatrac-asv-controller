// system headers
#include <chrono>
#include <cmath>

// other headers
#include <asv_utils/networking/UDPSender.h>
#include <asv_messages/AttitudeMessage.h>

// ros headers
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using namespace asv::networking;

#define LOCALHOST "127.0.0.2"
#define PORT 62002

class MockAttitude : public rclcpp::Node
{
public:
  MockAttitude() : Node("mock_attitude"), udp_sender_()
  {
    timer_ = this->create_wall_timer(250ms, std::bind(&MockAttitude::publish_data, this));
    this->declare_parameter("ip_address", LOCALHOST);
    this->declare_parameter("udp_port", PORT);
    udp_sender_.init(this->get_parameter("ip_address").as_string(), this->get_parameter("udp_port").as_int());
  }

private:
  void publish_data()
  {
    asv::messages::AttitudeMessage attitude_msg{};
    count_++;

    attitude_msg.pitch = std::sin(count_ * M_PI / 3);
    attitude_msg.min_pitch = -180;
    attitude_msg.min_pitch = 180;

    attitude_msg.roll = std::cos(count_ * M_PI / 3);
    attitude_msg.min_roll = -180;
    attitude_msg.min_roll = 180;

    attitude_msg.heading = std::sin(count_ * M_PI / 6);
    attitude_msg.min_heading = 0;
    attitude_msg.max_heading = 360;

    udp_sender_.send_message(attitude_msg.encode());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_{ 0 };
  asv::networking::UDPSender udp_sender_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  rclcpp::spin(std::make_shared<MockAttitude>());
  rclcpp::shutdown();
  return 0;
}
