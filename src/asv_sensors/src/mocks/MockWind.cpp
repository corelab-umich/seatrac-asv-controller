// system headers
#include <chrono>
#include <cmath>

// other headers
#include <asv_utils/networking/UDPSender.h>
#include <asv_messages/WindMessage.h>

// ros headers
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using namespace asv::networking;

#define LOCALHOST "127.0.0.2"
#define PORT 62002

class MockWind : public rclcpp::Node
{
public:
  MockWind() : Node("mock_wind"), udp_sender_()
  {
    timer_ = this->create_wall_timer(250ms, std::bind(&MockWind::publish_data, this));
    this->declare_parameter("ip_address", LOCALHOST);
    this->declare_parameter("udp_port", PORT);
    udp_sender_.init(this->get_parameter("ip_address").as_string(), this->get_parameter("udp_port").as_int());
  }

private:
  void publish_data()
  {
    asv::messages::WindMessage wind_msg{};
    count_++;

    wind_msg.apparent_speed = std::abs(std::sin(count_ * M_PI / 3));
    wind_msg.apparent_angle = std::cos(count_ * M_PI / 4);
    wind_msg.temperature = std::sin(count_ * M_PI / 5);
    wind_msg.pressure = std::cos(count_ * M_PI / 6);

    udp_sender_.send_message(wind_msg.encode());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_{ 0 };
  asv::networking::UDPSender udp_sender_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  rclcpp::spin(std::make_shared<MockWind>());
  rclcpp::shutdown();
  return 0;
}
