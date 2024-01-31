cmake_minimum_required(VERSION 3.8)
project(asv_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# only boost versions >=1.35.0 come bundled with the asio networking library
find_package(Boost 1.35.0 REQUIRED COMPONENTS system)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(asv_utils REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(imu_driver src/main.cpp)
# target_include_directories(imu_driver PRIVATE include/)
ament_target_dependencies(imu_driver PUBLIC rclcpp sensor_msgs Boost asv_utils)

install(TARGETS
  imu_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
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
