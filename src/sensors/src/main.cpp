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
using boost::asio::ip::address;
using boost::asio::ip::udp;

class MessageReceiver
{
public:
  virtual void handle_message(const char *buf, size_t buf_size) = 0;
};

class UDPClient
{
public:
  UDPClient(const std::string &ip_address, int port, size_t max_buffer_size) : recv_buffer_(max_buffer_size), ip_address_(ip_address), port_(port){};

  void start(MessageReceiver &handler_obj)
  {
    socket_.open(udp::v4());
    // socket_.bind(udp::endpoint(address::from_string(ip_address_), port_));
    socket_.bind(udp::endpoint(udp::v4(), port_));
    handlerObj = &handler_obj;

    async_receive_message();

    std::cout << "starting receiving\n";
    io_service_.run();
  }

private:
  void handle_receive(const boost::system::error_code &error, size_t bytes_transferred)
  {
    if (error)
    {
      std::cout << "Receive failed: " << error.message() << "\n";
    } else {
      handlerObj->handle_message(recv_buffer_.data(), bytes_transferred);
    }

    async_receive_message();
  }

  void async_receive_message()
  {
    socket_.async_receive_from(boost::asio::buffer(recv_buffer_),
                               remote_endpoint_,
                               boost::bind(&UDPClient::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
  }

  boost::asio::io_service io_service_{};
  udp::socket socket_{io_service_};
  std::vector<char> recv_buffer_;
  udp::endpoint remote_endpoint_;
  std::string ip_address_;
  int port_;
  MessageReceiver *handlerObj;
};

class Sender
{
private:
  boost::asio::io_service io_service_{};
  udp::socket socket_{io_service_};
  udp::endpoint remote_endpoint_{address::from_string(IPADDRESS), UDP_PORT};
  size_t count_;

public:
  Sender()
  {
    socket_.open(udp::v4());
  }

  void send_message(const std::string &message)
  {
    boost::system::error_code err;
    auto sent = socket_.send_to(boost::asio::buffer(message), remote_endpoint_, 0, err);
    std::cout << "Sent Payload #" << ++count_ << " --- " << sent << " bytes\n";
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
        1000ms, std::bind(&ImuDriver::publish_data, this));
  }

private:
  void publish_data()
  {

    auto msg = sensor_msgs::msg::Imu();
    msg.orientation.w = count_;
    imu_pub_->publish(msg);

    sender.send_message("message " + std::to_string(++count_));
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
  void handle_message(const char *buf, size_t buf_size)
  {
    std::cout << "Received message " << ++messages_received_ << ": '" << std::string(buf, buf + buf_size) << "'" << std::endl;
  }
private:
  size_t messages_received_{0};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  UDPClient client{std::string(IPADDRESS), UDP_PORT, BUFFER_SIZE};
  Test test{};
  std::thread r([&]
                { client.start(test); });
  rclcpp::spin(std::make_shared<ImuDriver>());
  rclcpp::shutdown();
  return 0;
}
