cmake_minimum_required(VERSION 3.8)
project(asv_messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(asv_utils REQUIRED)

# set(MESSAGES_HEADER_FILES include/messages/DatetimeMessage.h include/messages/ImuMessage.h)
set(MESSAGES_SRC_FILES src/DatetimeMessage.cpp src/ImuMessage.cpp src/PowerLevelMessage.cpp src/AttitudeMessage.cpp src/WindMessage.cpp)

add_library(asv_messages ${MESSAGES_SRC_FILES})
target_include_directories(asv_messages
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(asv_messages asv_utils)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

# extra stuff to make library visible in ros env
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS asv_messages
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(asv_utils)

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
