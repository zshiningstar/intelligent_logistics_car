cmake_minimum_required(VERSION 2.8.3)
project(euclidean_cluster)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
	pcl_ros
	roscpp
	geometry_msgs
	std_msgs
	sensor_msgs
	jsk_recognition_msgs
	message_generation
	tf
)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
	FILES
	ObjectPolygon.msg
	ObjectPolygonArray.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs 
	geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros std_msgs jsk_recognition_msgs geometry_msgs tf message_runtime
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node src/euclidean_cluster_node.cpp src/euclidean_cluster_core.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_node  euclidean_cluster_gencpp)

