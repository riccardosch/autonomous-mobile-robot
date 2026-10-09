cmake_minimum_required(VERSION 3.8)
project(lidar_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Frame decoding and serial port, without ROS: reused by the node and the tests
add_library(lidar_bridge_core
  src/frame_parser.cpp
  src/serial_port.cpp
)
target_include_directories(lidar_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(lidar_bridge_node src/lidar_bridge_node.cpp)
target_link_libraries(lidar_bridge_node
  lidar_bridge_core
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

install(TARGETS lidar_bridge_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS lidar_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_parser test/test_frame_parser.cpp)
  target_link_libraries(test_frame_parser lidar_bridge_core)
endif()

ament_package()
