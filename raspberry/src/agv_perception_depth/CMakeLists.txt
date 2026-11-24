cmake_minimum_required(VERSION 3.8)
project(agv_perception_depth)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)   # the inner loop runs on ~19k pixels at 10 Hz
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Depth -> points, without ROS: reused by the node and the tests
add_library(depth_to_points src/depth_to_points.cpp)
target_include_directories(depth_to_points PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(depth_obstacles src/depth_obstacles_node.cpp)
target_link_libraries(depth_obstacles
  depth_to_points
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
)

install(TARGETS depth_to_points ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS depth_obstacles DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_depth_to_points test/test_depth_to_points.cpp)
  target_link_libraries(test_depth_to_points depth_to_points)
endif()

ament_package()
