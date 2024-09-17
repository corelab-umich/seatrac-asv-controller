// system headers
#include <chrono>
#include <cmath>
#include <random>

// other headers
#include <asv_utils/networking/UDPSender.h>
#include <asv_messages/WindMessage.h>
#include <std_msgs/msg/header.hpp>
#include <rclcpp/rclcpp.hpp>
#include <messages/msg/wind.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

using namespace std::chrono_literals;


typedef struct
{
    double speed;
    double angle;
    double temperature; 
    double pressure;
} wind_data;



class MeasurementPacketPublisher : public rclcpp::Node
{
public:
  MeasurementPacketPublisher() : Node("measurement_packet_publisher")
  {
    wind_sub_ = this->create_subscription<::messages::msg::Wind>("/wind", rclcpp::SensorDataQoS(), std::bind(&MeasurementPacketPublisher::callback, this, std::placeholders::_1));

    measurement_pub_ = this->create_publisher<::messages::msg::Wind>("/measurement_packet", rclcpp::SensorDataQoS());

    timer_ = this->create_wall_timer(1000ms, std::bind(&MeasurementPacketPublisher::publish_data, this));
  }

private:

    void callback(const ::messages::msg::Wind ros_wind_msg)
    {
        
        // auto ros_wind_msg = asv::messages::WindMessage::decode(raw_data->data.data(), raw_data->data.size());
        // auto ros_wind_msg = raw_data;
        RCLCPP_DEBUG(this->get_logger(), "Wind Speed: %f", ros_wind_msg.apparent_speed);

        // RCLCPP_DEBUG(this->get_logger(), "Received Wind sensor message");
        RCLCPP_DEBUG(this->get_logger(), "Wind Speed: %f", ros_wind_msg.apparent_speed);

        //TODO: weight the constants min((sample_time/tau), 1)
        filtered_data.speed = (sample_time/tau) * ros_wind_msg.apparent_speed + (1 - (sample_time/tau)) * filtered_data.speed;
        filtered_data.angle = ros_wind_msg.apparent_angle;
        filtered_data.temperature = ros_wind_msg.temperature;
        filtered_data.pressure = ros_wind_msg.pressure;
    }

  void publish_data()
  {
    auto measurement_msg = ::messages::msg::Wind{};

    measurement_msg.header.stamp = this->now();
    measurement_msg.apparent_speed = filtered_data.speed;
    // measurement_msg.apparent_speed = tau;
    measurement_msg.apparent_angle = filtered_data.angle;
    measurement_msg.temperature = filtered_data.temperature;
    measurement_msg.pressure = filtered_data.pressure;

    measurement_pub_->publish(measurement_msg);
  }

    double sample_time = 0.25; // filter sample time in units of seconds
    double tau = 0.5; // filter time constant in units of seconds

    rclcpp::Subscription<::messages::msg::Wind>::SharedPtr wind_sub_;
  rclcpp::Publisher<::messages::msg::Wind>::SharedPtr measurement_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  wind_data filtered_data;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto mpp = std::make_shared<MeasurementPacketPublisher>();
  rclcpp::spin(mpp);
  rclcpp::shutdown();
  return 0;
}