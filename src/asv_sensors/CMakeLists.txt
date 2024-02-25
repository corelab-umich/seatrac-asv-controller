cmake_minimum_required(VERSION 3.8)
project(asv_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# only boost versions >=1.35.0 come bundled with the asio networking library
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(asv_utils REQUIRED)
find_package(tf2 REQUIRED)
find_package(asv_messages REQUIRED)
find_package(messages REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(mock_imu src/mocks/MockIMU.cpp)
ament_target_dependencies(mock_imu PUBLIC rclcpp asv_utils asv_messages)

add_executable(mock_power_level src/mocks/MockPowerLevel.cpp)
ament_target_dependencies(mock_power_level PUBLIC rclcpp asv_utils asv_messages)

add_executable(mock_attitude src/mocks/MockAttitude.cpp)
ament_target_dependencies(mock_attitude PUBLIC rclcpp asv_utils asv_messages)

add_executable(mock_gps src/mocks/MockGPS.cpp)
ament_target_dependencies(mock_gps PUBLIC rclcpp asv_utils asv_messages)

add_executable(mock_wind src/mocks/MockWind.cpp)
ament_target_dependencies(mock_wind PUBLIC rclcpp asv_utils asv_messages)

add_executable(packet_publisher src/SensorPacketPublisher.cpp include/asv_sensors/SensorPacketPublisher.h)
target_include_directories(packet_publisher PRIVATE include/)
ament_target_dependencies(packet_publisher PUBLIC rclcpp asv_utils std_msgs)

add_executable(imu_sensor_driver src/ImuSensorDriver.cpp include/asv_sensors/ImuSensorDriver.h)
target_include_directories(imu_sensor_driver PRIVATE include/)
ament_target_dependencies(imu_sensor_driver PUBLIC rclcpp asv_utils std_msgs sensor_msgs tf2 asv_messages)

add_executable(power_level_sensor_driver src/PowerLevelSensorDriver.cpp include/asv_sensors/PowerLevelSensorDriver.h)
target_include_directories(power_level_sensor_driver PRIVATE include/)
ament_target_dependencies(power_level_sensor_driver PUBLIC rclcpp std_msgs asv_messages messages)

add_executable(attitude_sensor_driver src/AttitudeSensorDriver.cpp include/asv_sensors/AttitudeSensorDriver.h)
target_include_directories(attitude_sensor_driver PRIVATE include/)
ament_target_dependencies(attitude_sensor_driver PUBLIC rclcpp std_msgs asv_messages messages)

add_executable(gps_sensor_driver src/GPSSensorDriver.cpp include/asv_sensors/GPSSensorDriver.h)
target_include_directories(gps_sensor_driver PRIVATE include/)
ament_target_dependencies(gps_sensor_driver PUBLIC rclcpp std_msgs asv_messages sensor_msgs messages)

add_executable(wind_sensor_driver src/WindSensorDriver.cpp include/asv_sensors/WindSensorDriver.h)
target_include_directories(wind_sensor_driver PRIVATE include/)
ament_target_dependencies(wind_sensor_driver PUBLIC rclcpp std_msgs asv_messages messages)

install(TARGETS
  packet_publisher
  imu_sensor_driver
  power_level_sensor_driver
  attitude_sensor_driver
  gps_sensor_driver
  wind_sensor_driver
  mock_imu
  mock_power_level
  mock_attitude
  mock_gps
  mock_wind
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
