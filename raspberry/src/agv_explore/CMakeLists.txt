cmake_minimum_required(VERSION 3.8)
project(agv_explore)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(tf2_ros REQUIRED)

# Exploration logic, without ROS: reused by the node and the tests
add_library(explore_core
  src/grid.cpp
  src/frontiers.cpp
  src/frontier_memory.cpp
  src/goal_selector.cpp
  src/map_file.cpp
)
target_include_directories(explore_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(explorer src/explorer_node.cpp)
target_link_libraries(explorer
  explore_core
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${slam_toolbox_TARGETS}
  tf2_ros::tf2_ros
)

install(TARGETS explore_core ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS explorer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t frontiers frontier_memory goal_selector map_file)
    ament_add_gtest(test_${t} test/test_${t}.cpp test/test_grid.hpp)
    target_link_libraries(test_${t} explore_core)
  endforeach()
endif()

ament_package()
