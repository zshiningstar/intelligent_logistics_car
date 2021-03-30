cmake_minimum_required(VERSION 3.0.2)
project(pub_odom)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  logistics_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES listener
#  CATKIN_DEPENDS roscpp rospy serial std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(pub_odom src/odom_pub.cpp)
add_dependencies(pub_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_odom ${catkin_LIBRARIES} )
