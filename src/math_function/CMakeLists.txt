cmake_minimum_required(VERSION 3.0.2)
project(math_function)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package(
  INCLUDE_DIRS include
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

