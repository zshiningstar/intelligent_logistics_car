cmake_minimum_required(VERSION 3.0.2)
project(planning)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  logistics_msgs
  ant_math
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planning
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(path_track src/path_track.cpp)
add_dependencies(path_track ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_track ${catkin_LIBRARIES} )
