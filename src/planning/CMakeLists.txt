cmake_minimum_required(VERSION 3.0.2)
project(planning)

 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  logistics_msgs
  math_function
)

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
