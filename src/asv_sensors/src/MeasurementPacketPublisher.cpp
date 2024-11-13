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
#include <asv_messages/PowerLevelMessage.h>

// Other Message Types
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <messages/msg/wind.hpp>
#include <messages/msg/sensor_data.hpp>
#include <messages/msg/asv_gps.hpp>
#include <messages/msg/power_level.hpp>

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

typedef struct
{
  double state_of_charge;
} pwr_data;


const float KTS_TO_MS = 0.5144444;
const double EARTH_RADIUS = 6371.0; // Radius of the Earth in kilometers
const double DEG_TO_RAD = M_PI / 180.0; // Convert degrees to radians
const double ASV_MAX_BATTERY = 6500.00; // Max capacity of ASV battery in Wh


class MeasurementPacketPublisher : public rclcpp::Node
{
public:
  MeasurementPacketPublisher() : Node("measurement_packet_publisher")
  {
    // Declare Domain Origin Parameter
    // this->declare_parameter("origin_latitude", 35.751066);
    // this->declare_parameter("origin_longitude", -79.034666);
    this->declare_parameter("origin_latitude", 35.703543);
    this->declare_parameter("origin_longitude", -79.042890);
    this->declare_parameter("rated_wind_speed_kts", 1.5);

    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
                      .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    wind_sub_ = this->create_subscription<::messages::msg::Wind>("/wind", rclcpp::SensorDataQoS(), std::bind(&MeasurementPacketPublisher::wind_parser, this, std::placeholders::_1));
    gps_sub_ = this->create_subscription<::messages::msg::AsvGps>("/gps", rclcpp::SensorDataQoS(), std::bind(&MeasurementPacketPublisher::gps_parser, this, std::placeholders::_1));
    power_sub_ = this->create_subscription<::messages::msg::PowerLevel>("/power_level", rclcpp::SensorDataQoS(), std::bind(&MeasurementPacketPublisher::soc_parser, this, std::placeholders::_1));

    measurement_pub_ = this->create_publisher<::messages::msg::SensorData>("/measurement_packet", qos_profile);

    timer_ = this->create_wall_timer(1000ms, std::bind(&MeasurementPacketPublisher::publish_data, this));
  }

private:

    void wind_parser(const ::messages::msg::Wind ros_wind_msg)
    {        
        // double filter_weight = std::min((sample_time/tau), double(1));
        // parsed_wind.speed = filter_weight * ros_wind_msg.apparent_speed + (1 - filter_weight) * parsed_wind.speed;
        // TODO: Convert to desired distribution
        parsed_wind.speed = ros_wind_msg.apparent_speed * KTS_TO_MS;
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

    void soc_parser(const ::messages::msg::PowerLevel pwr_lvl_msg)
    {
      parsed_pwr.state_of_charge = (pwr_lvl_msg.soc_percent/100.00) * ASV_MAX_BATTERY; 
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

    // GPS Position of ASV
    measurement_msg.latitude = parsed_gps.latitude;
    measurement_msg.longitude = parsed_gps.longitude;

    // ASV State of Charge
    measurement_msg.stateofcharge = parsed_pwr.state_of_charge;

    // Current heading and speed (as returned by GPS - requires ASV to move)
    measurement_msg.courseoverground = parsed_gps.heading;
    measurement_msg.speedoverground = parsed_gps.speed;

    // Wind Measurement
    measurement_msg.apparentwindspeed = parsed_wind.speed;
    measurement_msg.apparentwindangle = parsed_wind.angle;
    double true_wind_speed = calculateTrueWindSpeed(parsed_wind.speed, parsed_wind.angle, parsed_gps.speed, parsed_gps.heading);

    measurement_msg.truewind = true_wind_speed;

    // Adjust the true wind speed to the normalized distribution
    if(std::isnan(raw_to_normal(true_wind_speed))){
      measurement_msg.windspeed = -1.0;
    }else{
      measurement_msg.windspeed = raw_to_normal(true_wind_speed);
    }
    

    // Adjust the rated true wind speed by subtracting the running average
    double rated_wind_ms = this->get_parameter("rated_wind_speed_kts").as_double() * KTS_TO_MS;
    measurement_msg.ratedwind = raw_to_normal(rated_wind_ms);

    measurement_pub_->publish(measurement_msg);
  }

  // Function to convert raw wind speed to normal distribution for KF
  double raw_to_normal(double raw_wind){
    double lambda = 5.5781970;
    double k = 2.3007;
    double mu = 0.0;
    double std_dev = 1.0;
    return inv_gaussian(weibull(raw_wind, lambda, k), mu, std_dev);
  }

  double weibull(double x, double lambda, double k){
    double result = 1.0 - exp(-1.0 * pow(x/lambda, k));
    return result;
  }

  double inv_gaussian(double x, double mu, double std_dev){
    double result = mu + sqrt(2.0 * pow(std_dev, 2.0)) * erfinv(-1.0 + 2.0*x);
    return result;
  }

  double erfinv(double x) {
    // Coefficients for approximation
    const double a[] = {0.886226899, -1.645349621, 0.914624893, -0.140543331};
    const double b[] = {-2.118377725, 1.442710462, -0.329097515, 0.012229801};
    const double c[] = {-1.970840454, -1.62490649, 3.429567803, 1.641345311};
    const double d[] = {3.543889200, 1.637067800};

    // Bounds for the approximation accuracy
    if (x < -1.0 || x > 1.0) {
        throw std::domain_error("erfinv(x) only defined for -1 <= x <= 1");
    }
    
    if (x == 0.0) return 0.0;
    if (x == 1.0) return std::numeric_limits<double>::infinity();
    if (x == -1.0) return -std::numeric_limits<double>::infinity();
    
    double sign = (x > 0) ? 1.0 : -1.0;
    double ln_term = std::log(1.0 - x * x);

    double result;
    if (std::abs(x) <= 0.7) {
        double sum_num = (((a[3] * x + a[2]) * x + a[1]) * x + a[0]) * x;
        double sum_den = (((b[3] * x + b[2]) * x + b[1]) * x + b[0]) * x + 1.0;
        result = sum_num / sum_den;
    } else {
        double sum_num = ((c[3] * ln_term + c[2]) * ln_term + c[1]) * ln_term + c[0];
        double sum_den = (d[1] * ln_term + d[0]) * ln_term + 1.0;
        result = sign * std::sqrt(sum_num / sum_den);
    }

    return result;
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

  // Function to calculate the true wind speed from apparent wind and boat speed
  double calculateTrueWindSpeed(double apparentWindSpeed, double apparentWindAngle,
                              double boatSpeed, double boatCourse) {

    // Convert angles from degrees to radians
    double AWA_rad = apparentWindAngle * M_PI/180.0;
    double HDG_rad = boatCourse * M_PI/180.0;

    // Calculate the apparent wind components
    double AW_x = apparentWindSpeed * cos(AWA_rad + HDG_rad);
    double AW_y = apparentWindSpeed * sin(AWA_rad + HDG_rad);

    // Calculate boat velocity components
    double BV_x = boatSpeed * cos(HDG_rad);
    double BV_y = boatSpeed * sin(HDG_rad);

    // Calculate true wind components
    double TW_x = AW_x - BV_x;
    double TW_y = AW_y - BV_y;

    double TWS = sqrt(TW_x * TW_x + TW_y * TW_y);
    return TWS;
}

  double ns_distance;
  double ew_distance;

  rclcpp::Subscription<::messages::msg::Wind>::SharedPtr wind_sub_;
  rclcpp::Subscription<::messages::msg::AsvGps>::SharedPtr gps_sub_;
  rclcpp::Subscription<::messages::msg::PowerLevel>::SharedPtr power_sub_;
  rclcpp::Publisher<::messages::msg::SensorData>::SharedPtr measurement_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  wind_data parsed_wind;
  gps_data parsed_gps;
  pwr_data parsed_pwr;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto mpp = std::make_shared<MeasurementPacketPublisher>();
  rclcpp::spin(mpp);
  rclcpp::shutdown();
  return 0;
}