/*
Node to read IMU sensor data from UDP packets and publish data on the following topics:
/imu/std_data - topic for standard sensor_msgs/Imu message
/imu/full_data -  topic for custom message containing the full data published by IMU
*/

// system headers
#include <chrono>
#include <cmath>

// other headers
#include <asv_utils/networking/UDPSender.h>
#include <asv_messages/ImuMessage.h>

// ros headers
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using namespace asv::networking;

#define LOCALHOST "127.0.0.1"
#define PORT 62001

class MockIMU : public rclcpp::Node
{
public:
  MockIMU()
      : Node("mock_imu"), udp_sender_(LOCALHOST, PORT)
  {
    // imu_pub_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("raw_sensor_packets", rclcpp::SensorDataQoS());
    timer_ = this->create_wall_timer(
        250ms, std::bind(&MockIMU::publish_data, this));
  }

private:
  void publish_data()
  {
    asv::messages::ImuMessage imu_msg{};
    count_++;

    imu_msg.roll = std::sin(count_ * M_PI / 6);
    imu_msg.pitch = std::cos(count_ * M_PI / 6);
    imu_msg.heading = std::abs(std::sin(count_ * M_PI / 180));

    imu_msg.acceleration_x = std::sin(count_ * M_PI / 6);
    imu_msg.acceleration_y = std::cos(count_ * M_PI / 6);
    imu_msg.acceleration_z = std::abs(std::sin(count_ * M_PI / 180));

    imu_msg.roll_gyro_rate = std::sin(count_ * M_PI / 6);
    imu_msg.pitch_gyro_rate = std::cos(count_ * M_PI / 6);
    imu_msg.heading_gyro_rate = std::abs(std::sin(count_ * M_PI / 180));

    udp_sender_.send_message(imu_msg.encode());
  }

  // rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_{0};
  asv::networking::UDPSender udp_sender_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  rclcpp::spin(std::make_shared<MockIMU>());
  rclcpp::shutdown();
  return 0;
}
