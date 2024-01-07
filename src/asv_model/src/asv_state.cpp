/* 
Node to pull aggregate information from all sensors and repackage 
into a single SensorData msg  
*/

// ros headers
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "messages/msg/sensor_data.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using namespace std::chrono_literals;


class ASVState : public rclcpp::Node
{
public:
  ASVState()
  : Node("asv_state"), count_(0)
  {
    publisher_ = this->create_publisher<messages::msg::SensorData>("sensor_data", 10);
    nav_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("nav_data", 10);
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&ASVState::publish_data, this));
  }

private:

  void publish_data()
  {
    auto msg_out = messages::msg::SensorData();
    msg_out.latitude = 35.0 - 0.25*count_;
    msg_out.longitude = -75.5 + 0.25*count_;
    msg_out.stateofcharge = 99.830;
    publisher_->publish(msg_out);

    auto nav_msg = sensor_msgs::msg::NavSatFix();
    nav_msg.latitude = 35.0 - 0.25*count_;
    nav_msg.longitude = -75.5 + 0.25*count_;
    nav_pub_->publish(nav_msg);

    count_++;
  }

  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<messages::msg::SensorData>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_pub_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ASVState>());
  rclcpp::shutdown();
  return 0;
}
