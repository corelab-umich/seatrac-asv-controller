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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(mock_imu src/MockIMU.cpp)
ament_target_dependencies(mock_imu PUBLIC rclcpp asv_utils asv_messages)

add_executable(packet_publisher src/SensorPacketPublisher.cpp include/asv_sensors/SensorPacketPublisher.h)
target_include_directories(packet_publisher PRIVATE include/)
ament_target_dependencies(packet_publisher PUBLIC rclcpp asv_utils std_msgs)

add_executable(imu_sensor_driver src/ImuSensorDriver.cpp include/asv_sensors/ImuSensorDriver.h)
target_include_directories(imu_sensor_driver PRIVATE include/)
ament_target_dependencies(imu_sensor_driver PUBLIC rclcpp asv_utils std_msgs sensor_msgs tf2 asv_messages)

install(TARGETS
  mock_imu
  packet_publisher
  imu_sensor_driver
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
