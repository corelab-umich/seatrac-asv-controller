// system headers
#include <chrono>
#include <cmath>

// other headers
#include <asv_utils/networking/UDPSender.h>
#include <asv_messages/GPSMessage.h>

// ros headers
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using namespace asv::networking;

#define LOCALHOST "127.0.0.2"
#define PORT 62002

class MockGPS : public rclcpp::Node
{
public:
  MockGPS() : Node("mock_gps"), udp_sender_()
  {
    timer_ = this->create_wall_timer(250ms, std::bind(&MockGPS::publish_data, this));
    this->declare_parameter("ip_address", LOCALHOST);
    this->declare_parameter("udp_port", PORT);
    udp_sender_.init(this->get_parameter("ip_address").as_string(), this->get_parameter("udp_port").as_int());
  }

private:
  void publish_data()
  {
    asv::messages::GPSMessage gps_msg{};
    count_++;

    gps_msg.latitude = 29.9792 * M_PI / 180.0;
    gps_msg.longitude = 31.1342 * M_PI / 180.0;
    gps_msg.kts = std::abs(std::sin(count_ * M_PI / 3));
    gps_msg.heading = std::abs(std::cos(count_ * M_PI / 4));
    gps_msg.current_kts = std::abs(std::sin(count_ * M_PI / 5));
    gps_msg.current_heading = std::abs(std::cos(count_ * M_PI / 6));
    gps_msg.wind_kts = std::abs(std::sin(count_ * M_PI / 3));
    gps_msg.wind_heading = std::abs(std::cos(count_ * M_PI / 4));

    udp_sender_.send_message(gps_msg.encode());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_{ 0 };
  asv::networking::UDPSender udp_sender_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  rclcpp::spin(std::make_shared<MockGPS>());
  rclcpp::shutdown();
  return 0;
}
