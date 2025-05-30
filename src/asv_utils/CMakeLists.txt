cmake_minimum_required(VERSION 3.8)
project(asv_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

# find dependencies
find_package(ament_cmake REQUIRED)
include(ConfigExtras)

set(UTIL_HEADER_FILES include/asv_utils/utils/utils.h)
set(NETWORKING_HEADER_FILES include/asv_utils/networking/UDPClient.h include/asv_utils/networking/UDPSender.h)

set(UTIL_SRC_FILES src/utils/utils.cpp)
set(NETWORKING_SRC_FILES src/networking/UDPClient.cpp src/networking/UDPSender.cpp)

add_library(asv_utils ${UTIL_SRC_FILES} ${NETWORKING_SRC_FILES})
target_include_directories(asv_utils
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(asv_utils Boost)

# extra stuff to make library visible in ros env
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS asv_utils
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost)

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

ament_package(CONFIG_EXTRAS "cmake/ConfigExtras.cmake")
