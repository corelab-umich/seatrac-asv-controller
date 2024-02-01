/*
Node to read IMU sensor data from UDP packets and publish data on the following topics:
/imu/std_data - topic for standard sensor_msgs/Imu message
/imu/full_data -  topic for custom message containing the full data published by IMU
*/

// system headers
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

// other headers
#include <asv_utils/utils/utils.h>
#include <asv_utils/networking/UDPClient.h>
#include <asv_utils/networking/MessageReceiver.h>

// ros headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

// networking headers
#include <boost/asio.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>

#define IPADDRESS "127.0.0.1"
#define UDP_PORT 13251
#define BUFFER_SIZE 1024

using namespace std::chrono_literals;
using namespace asv::networking;
using namespace asv::utils;
using boost::asio::ip::address;
using boost::asio::ip::udp;


// this is just part of test infra for now. maybe will convert into proper test lib later
class Sender
{
private:
  boost::asio::io_service io_service_{};
  udp::socket socket_{io_service_};
  udp::endpoint remote_endpoint_{address::from_string(IPADDRESS), UDP_PORT};
  unsigned char count_;

public:
  Sender()
  {
    socket_.open(udp::v4());
  }

  void send_message(const std::string &)
  {
    boost::system::error_code err;
    std::vector<unsigned char> c_ar{};
    c_ar.push_back(count_);
    c_ar.resize(1);
    auto sent = socket_.send_to(boost::asio::buffer(c_ar), remote_endpoint_, 0, err);
    std::cout << "Sent Payload # " << (int)++count_ << " --- " << sent << " bytes\n";
  }
};

class ImuDriver : public rclcpp::Node
{
public:
  ImuDriver()
      : Node("imu_driver")
  {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("sensor_data", 1);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&ImuDriver::publish_data, this));
  }

private:
  void publish_data()
  {

    auto msg = sensor_msgs::msg::Imu();
    msg.orientation.w = count_;
    imu_pub_->publish(msg);

    sender.send_message("");
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  boost::asio::io_service io_service_{};
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  Sender sender{};
};

class Test: public MessageReceiver
{
public:
  void handle_message(const unsigned char *buf, size_t buf_size)
  {
    int num = buf[0];
    std::cout << "Received message " << ++messages_received_ << " of size " << buf_size << ": '" << num << "'" << std::endl;
  }
private:
  size_t messages_received_{0};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  UDPClient client{std::string(IPADDRESS), UDP_PORT, BUFFER_SIZE};
  Test test{};
  // std::thread r([&]
  //               { client.start(test); });
  // auto n = std::make_shared<ImuDriver>();
  rclcpp::spin(std::make_shared<ImuDriver>());
  // while (rclcpp::ok()) {
  //   std::this_thread::sleep_for(100ms);
  // }
  rclcpp::shutdown();
  return 0;
}
