// system headers
#include <chrono>
#include <cmath>
#include <random>

// other headers
#include <rclcpp/rclcpp.hpp>
#include <asv_utils/networking/UDPSender.h>

// ASV Message Types
#include <asv_messages/WindMessage.h>
#include <asv_messages/GPSMessage.h>

// Other Message Types
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <messages/msg/wind.hpp>
#include <messages/msg/sensordata.hpp>
#include <messages/msg/asvgps.hpp>

using namespace std::chrono_literals;


typedef struct
{
    double speed;
    double angle;
    double temperature; 
    double pressure;
} wind_data;

typedef struct
{
    double latitude;
    double longitude;
    double speed;
    double heading;
} gps_data;

const float KTS_TO_MS = 0.5144444;


class MeasurementPacketPublisher : public rclcpp::Node
{
public:
  MeasurementPacketPublisher() : Node("measurement_packet_publisher")
  {
    wind_sub_ = this->create_subscription<::messages::msg::Wind>("/wind", rclcpp::SensorDataQoS(), std::bind(&MeasurementPacketPublisher::wind_parser, this, std::placeholders::_1));
    gps_sub_ = this->create_subscription<::messages::msg::AsvGps>("/gps", rclcpp::SensorDataQoS(), std::bind(&MeasurementPacketPublisher::gps_parser, this, std::placeholders::_1));

    measurement_pub_ = this->create_publisher<::messages::msg::Wind>("/measurement_packet", rclcpp::SensorDataQoS());

    timer_ = this->create_wall_timer(1000ms, std::bind(&MeasurementPacketPublisher::publish_data, this));
  }

private:

    void wind_parser(const ::messages::msg::Wind ros_wind_msg)
    {
        
        // auto ros_wind_msg = asv::messages::WindMessage::decode(raw_data->data.data(), raw_data->data.size());
        // auto ros_wind_msg = raw_data;
        RCLCPP_DEBUG(this->get_logger(), "Wind Speed: %f", ros_wind_msg.apparent_speed);

        // RCLCPP_DEBUG(this->get_logger(), "Received Wind sensor message");
        RCLCPP_DEBUG(this->get_logger(), "Wind Speed: %f", ros_wind_msg.apparent_speed);

        double filter_weight = std::min((sample_time/tau), double(1));
        parsed_wind.speed = filter_weight * ros_wind_msg.apparent_speed + (1 - filter_weight) * parsed_wind.speed;
        parsed_wind.angle = ros_wind_msg.apparent_angle;
        parsed_wind.temperature = ros_wind_msg.temperature;
        parsed_wind.pressure = ros_wind_msg.pressure;
    }

    void gps_parser(const :: messages::msg::AsvGps ros_gps_msg)
    {
      parsed_gps.latitude = ros_gps_msg.latitude;
      parsed_gps.longitude = ros_gps_msg.longitude;

      parsed_gps.speed = KTS_TO_MS * ros_gps_msg.current_kts; // Convert speed in kts to m/s
      paresed_gps.heading = ros_gps_msg.current_heading;
    }

  void publish_data()
  {
    auto measurement_msg = ::messages::msg::SensorData{};

    measurement_msg.windspeed = parsed_wind.speed;
    measurement_msg.latitude = parsed_gps.latitude;
    measurement_msg.longitude = parsed_gps.longitude;
    measurement_msg.courseoverground = parsed_gps.heading;
    measurement_msg.speedoverground = parsed_gps.speed;

    measurement_pub_->publish(measurement_msg);
  }

  double sample_time = 0.25; // filter sample time in units of seconds
  double tau = 0.5; // filter time constant in units of seconds

  rclcpp::Subscription<::messages::msg::Wind>::SharedPtr wind_sub_;
  rclcpp::Publisher<::messages::msg::Wind>::SharedPtr measurement_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  wind_data parsed_wind;
  gps_data parsed_gps;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto mpp = std::make_shared<MeasurementPacketPublisher>();
  rclcpp::spin(mpp);
  rclcpp::shutdown();
  return 0;
}