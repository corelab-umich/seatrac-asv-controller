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

unsigned short to_ushort(const unsigned char *buf, bool little_endian = true) {
  unsigned short out = 0;
  if (little_endian) {
    out = buf[1];
    out <<= 8;
    out |= buf[0];
  } else {
    out = buf[0];
    out <<= 8;
    out |= buf[1];
  }

  return out;
}

short to_short(const unsigned char *buf, bool little_endian = true) {
  short out = 0;
  if (little_endian) {
    out = buf[1];
    out <<= 8;
    out |= buf[0];
  } else {
    out = buf[0];
    out <<= 8;
    out |= buf[1];
  }

  return out;
}

class MessageReceiver
{
public:
  virtual void handle_message(const unsigned char *buf, size_t buf_size) = 0;
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
  std::vector<unsigned char> recv_buffer_;
  udp::endpoint remote_endpoint_;
  std::string ip_address_;
  int port_;
  MessageReceiver *handlerObj;
};

struct DatetimeMessage {
  // Year
  unsigned short year;
  // Month: Jan = 1, Dec = 12
  unsigned short month;
  // Day: 1 to 31
  unsigned short day;
  // Hour: 0 to 23
  unsigned short hour;
  // Minute: 0 to 59
  unsigned short minute;
  // Second: 0 to 59
  unsigned short second;
  // Hundredths: 0 to 99
  unsigned short hundreths;

  static DatetimeMessage decode(const unsigned char *buf, size_t buf_len) {
    if (buf_len != 8) {
      throw std::invalid_argument("incorrect buffer size. must be 8 bytes");
    }
    DatetimeMessage out{};
    out.year = to_ushort(buf);
    out.month = buf[2];
    out.day = buf[3];
    out.hour = buf[4];
    out.minute = buf[5];
    out.second = buf[6];
    out.hundreths = buf[7];
    
    std::cout << "year: " << out.year;
    return out;
  }
};

struct ImuMessage {
  unsigned short sink_id;
  DatetimeMessage timestamp;

  // units for RPY: ??? (guessing degrees)
  float roll;
  float min_roll;
  float max_roll;
  float pitch;
  float min_pitch;
  float max_pitch;
  float heading;
  float min_heading;
  float max_heading;

  // gyro rates: deg/sec
  float roll_gyro_rate;
  float pitch_gyro_rate;
  float heading_gyro_rate;

  // accelerations: m/s^2
  // x: forwards
  // y: starboard
  // z: down 
  float acceleration_x;
  float acceleration_y;
  float acceleration_z;
  float max_acceleration_x;
  float max_acceleration_y;
  float max_acceleration_z;

  // z (heave): meters
  float z;
  float min_z;
  float max_z;

  // data size of udp packet send by boat
  static const size_t buffer_size{59};

  static ImuMessage decode(const unsigned char *buf, size_t buf_len) {
    if (buf_len != ImuMessage::buffer_size || buf[0] != 0 || buf[1] != 0xff) {
      throw std::invalid_argument("failed to decode");
    }
    ImuMessage out{};
    out.sink_id = buf[6];
    out.timestamp = DatetimeMessage::decode(buf + 7, 8);
    out.roll = 0.01 * to_short(buf + 15);
    out.min_roll = 0.01 * to_short(buf + 17);
    out.max_roll = 0.01 * to_short(buf + 19);
    out.pitch = 0.01 * to_short(buf + 21);
    out.min_pitch = 0.01 * to_short(buf + 23);
    out.max_pitch = 0.01 * to_short(buf + 25);
    out.heading = 0.01 * to_ushort(buf + 27);
    out.roll_gyro_rate = 0.02 * to_short(buf + 29);
    out.pitch_gyro_rate = 0.02 * to_short(buf + 31);
    out.heading_gyro_rate = 0.02 * to_short(buf + 33);
    out.acceleration_x = 0.01 * to_short(buf + 35);
    out.acceleration_y = 0.01 * to_short(buf + 37);
    out.acceleration_z = 0.01 * to_short(buf + 39);
    out.max_acceleration_x = 0.01 * to_short(buf + 41);
    out.max_acceleration_y = 0.01 * to_short(buf + 43);
    out.max_acceleration_z = 0.01 * to_short(buf + 45);
    out.z = 0.001 * to_short(buf + 47);
    out.min_z = 0.001 * to_short(buf + 49);
    out.max_z = 0.001 * to_short(buf + 51);
    out.min_heading = 0.01 * to_ushort(buf + 53);
    out.max_heading = 0.01 * to_ushort(buf + 55);

    return out;
  }

};

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
        1000ms, std::bind(&ImuDriver::publish_data, this));
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
  std::thread r([&]
                { client.start(test); });
  rclcpp::spin(std::make_shared<ImuDriver>());
  rclcpp::shutdown();
  return 0;
}
