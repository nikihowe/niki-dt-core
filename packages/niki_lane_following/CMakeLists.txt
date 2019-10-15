cmake_minimum_required(VERSION 2.8.3)
project(niki_lane_following)

find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  cv_bridge
  tf
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
