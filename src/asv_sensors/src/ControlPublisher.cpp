// system headers
#include <chrono>
#include <cmath>
#include <random>
#include <vector>

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

// A struct to hold each data point (RPM, Speed in knots)
struct DataPoint {
    float rpm;
    float speed;
};

// Table data
std::vector<DataPoint> table = {
    {0, 0.0}, {200, 0.7}, {400, 1.6}, {500, 2.0},
    {600, 2.5}, {700, 2.9}, {800, 3.3}, {900, 3.6},
    {1000, 4.0}, {1100, 4.3}, {1200, 4.6}, {1300, 5.0}
};


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

      // Ensure the target speed is within the range of the table
      if (kts <= table.front().speed) return table.front().rpm;
      if (kts >= table.back().speed) return table.back().rpm;

      // Find the two data points that bracket the target speed
      for (size_t i = 0; i < table.size() - 1; ++i) {
          if (table[i].speed <= kts && kts <= table[i + 1].speed) {
              // Manual linear interpolation for RPM
              double ratio = (kts - table[i].speed) / (table[i + 1].speed - table[i].speed);
              return  float(static_cast<int>(table[i].rpm + ratio * (table[i + 1].rpm - table[i].rpm)));
          }
      }
      // Return -1 if something goes wrong (shouldn't happen with valid data)
      return -1;
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