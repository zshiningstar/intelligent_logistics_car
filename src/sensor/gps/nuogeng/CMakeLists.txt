cmake_minimum_required(VERSION 2.8.3)
project(nuogeng)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -I /usr/include/eigen3)

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
#  LIBRARIES nuogeng
#  CATKIN_DEPENDS message_generation roscpp rospy serial std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/nuogeng_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

add_executable(record_gps_satellite src/record_gps_satellite.cpp)
add_dependencies(record_gps_satellite ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_gps_satellite ${catkin_LIBRARIES} )

add_executable(satellite_node src/satellite_node.cpp)
add_dependencies(satellite_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(satellite_node ${catkin_LIBRARIES} )

add_executable(record_gps_node src/record_path.cpp)
add_dependencies(record_gps_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_gps_node ${catkin_LIBRARIES} )
