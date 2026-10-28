cmake_minimum_required(VERSION 3.8)
project(kinect_driver)

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
# Installed by libfreenect in /usr/local/share/libfreenect/libfreenectConfig.cmake:
# it only sets FREENECT_INCLUDE_DIRS, FREENECT_LIBRARY_DIRS and FREENECT_LIBRARIES.
find_package(libfreenect REQUIRED)

add_executable(kinect_driver_node src/kinect_driver_node.cpp)
target_include_directories(kinect_driver_node PRIVATE ${FREENECT_INCLUDE_DIRS})
target_link_directories(kinect_driver_node PRIVATE ${FREENECT_LIBRARY_DIRS})
target_link_libraries(kinect_driver_node
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${FREENECT_LIBRARIES}
)

install(TARGETS kinect_driver_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
