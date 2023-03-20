cmake_minimum_required(VERSION 2.8.3)
project(lidar_crop)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs nav_msgs genmsg   geometry_msgs pcl_conversions pcl_ros visualization_msgs tf) 

## Declare ROS messages and services

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs visualization_msgs nav_msgs geometry_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(lidarcrop src/lidarcrop.cpp)
target_link_libraries(lidarcrop ${catkin_LIBRARIES})
add_dependencies(lidarcrop lidar_crop_generate_messages_cpp)

