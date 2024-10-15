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
#include <messages/msg/sensor_data.hpp>
#include <messages/msg/asv_gps.hpp>

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
const double EARTH_RADIUS = 6371.0; // Radius of the Earth in kilometers
const double DEG_TO_RAD = M_PI / 180.0; // Convert degrees to radians


class MeasurementPacketPublisher : public rclcpp::Node
{
public:
  MeasurementPacketPublisher() : Node("measurement_packet_publisher")
  {
    // Declare Domain Origin Parameter
    this->declare_parameter("origin_latitude", 35.751066);
    this->declare_parameter("origin_longitude", -79.034666);

    wind_sub_ = this->create_subscription<::messages::msg::Wind>("/wind", rclcpp::SensorDataQoS(), std::bind(&MeasurementPacketPublisher::wind_parser, this, std::placeholders::_1));
    gps_sub_ = this->create_subscription<::messages::msg::AsvGps>("/gps", rclcpp::SensorDataQoS(), std::bind(&MeasurementPacketPublisher::gps_parser, this, std::placeholders::_1));

    measurement_pub_ = this->create_publisher<::messages::msg::SensorData>("/measurement_packet", rclcpp::SensorDataQoS());

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

    void gps_parser(const ::messages::msg::AsvGps ros_gps_msg)
    {
      parsed_gps.latitude = ros_gps_msg.latitude;
      parsed_gps.longitude = ros_gps_msg.longitude;

      parsed_gps.speed = KTS_TO_MS * ros_gps_msg.current_kts; // Convert speed in kts to m/s
      parsed_gps.heading = ros_gps_msg.current_heading;
    }

  void publish_data()
  {
    auto measurement_msg = ::messages::msg::SensorData{};

    // Calculate Position in x/y coordinates
    double origin_lat = this->get_parameter("origin_latitude").as_double();
    double origin_long = this->get_parameter("origin_longitude").as_double();
    calculateDistance(origin_lat, origin_long, parsed_gps.latitude, parsed_gps.longitude, ns_distance, ew_distance);
    measurement_msg.pose_y = ns_distance;
    measurement_msg.pose_x = ew_distance;

    measurement_msg.windspeed = parsed_wind.speed;
    measurement_msg.latitude = parsed_gps.latitude;
    measurement_msg.longitude = parsed_gps.longitude;
    measurement_msg.courseoverground = parsed_gps.heading;
    measurement_msg.speedoverground = parsed_gps.speed;

    measurement_pub_->publish(measurement_msg);
  }

  // Function to calculate the N-S and E-W distance
  void calculateDistance(double lat_origin, double lon_origin, double lat, double lon, double& ns_distance, double& ew_distance) {
      // Convert latitudes and longitudes from degrees to radians
      lat_origin *= DEG_TO_RAD;
      lon_origin *= DEG_TO_RAD;
      lat *= DEG_TO_RAD;
      lon *= DEG_TO_RAD;

      // North-South distance relative to the origin
      ns_distance = EARTH_RADIUS * (lat - lat_origin);

      // East-West distance relative to the origin (account for the change in longitude at different latitudes)
      double avg_lat = (lat + lat_origin) / 2.0;
      ew_distance = EARTH_RADIUS * cos(avg_lat) * (lon - lon_origin);
  }

  double sample_time = 0.25; // filter sample time in units of seconds
  double tau = 0.5; // filter time constant in units of seconds
  double ns_distance;
  double ew_distance;

  rclcpp::Subscription<::messages::msg::Wind>::SharedPtr wind_sub_;
  rclcpp::Subscription<::messages::msg::AsvGps>::SharedPtr gps_sub_;
  rclcpp::Publisher<::messages::msg::SensorData>::SharedPtr measurement_pub_;
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