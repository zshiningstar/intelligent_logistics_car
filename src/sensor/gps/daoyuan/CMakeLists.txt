cmake_minimum_required(VERSION 3.0.2)
project(daoyuan)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  gps_msgs
  message_filters
  geodesy
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES daoyuan
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(daoyuan src/daoyuan.cpp)
add_dependencies(daoyuan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(daoyuan ${catkin_LIBRARIES} )
