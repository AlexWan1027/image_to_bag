cmake_minimum_required(VERSION 2.8.3)
project(image_to_bag)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w -g")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  roslib
  sensor_msgs
  std_msgs
  cv_bridge
)
find_package(OpenCV REQUIRED)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/image_to_bag_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)