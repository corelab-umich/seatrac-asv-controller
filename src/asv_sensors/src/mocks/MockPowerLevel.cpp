// system headers
#include <chrono>
#include <cmath>

// other headers
#include <asv_utils/networking/UDPSender.h>
#include <asv_messages/PowerLevelMessage.h>

// ros headers
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using namespace asv::networking;

#define LOCALHOST "127.0.0.2"
#define PORT 62002

class MockPowerLevel : public rclcpp::Node
{
public:
  MockPowerLevel() : Node("mock_power_level"), udp_sender_()
  {
    timer_ = this->create_wall_timer(250ms, std::bind(&MockPowerLevel::publish_data, this));
    this->declare_parameter("ip_address", LOCALHOST);
    this->declare_parameter("udp_port", PORT);
    udp_sender_.init(this->get_parameter("ip_address").as_string(), this->get_parameter("udp_port").as_int());
  }

private:
  void publish_data()
  {
    asv::messages::PowerLevelMessage power_msg{};
    count_++;

    power_msg.pack_current = std::sin(count_ * M_PI / 6);
    power_msg.load_current = std::cos(count_ * M_PI / 6);
    power_msg.pack_voltage = std::abs(std::sin(count_ * M_PI / 180));
    power_msg.soc_percent = std::abs(std::cos(count_ * M_PI / 180));

    udp_sender_.send_message(power_msg.encode());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_{ 0 };
  asv::networking::UDPSender udp_sender_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  rclcpp::spin(std::make_shared<MockPowerLevel>());
  rclcpp::shutdown();
  return 0;
}
