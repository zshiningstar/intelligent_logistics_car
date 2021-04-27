cmake_minimum_required(VERSION 3.0.2)
project(pc_stm32)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  logistics_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES listener
#  CATKIN_DEPENDS roscpp rospy serial std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
 lib
  ${catkin_INCLUDE_DIRS}
)

add_executable(pc_stm32 src/pc_stm32.cpp)
add_dependencies(pc_stm32 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pc_stm32 ${catkin_LIBRARIES} )

#add_executable(talk_stm32 src/talk_stm32.cpp)
#add_dependencies(talk_stm32 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(talk_stm32 ${catkin_LIBRARIES} )

#add_executable(try src/try.cpp)
#add_dependencies(try ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(try ${catkin_LIBRARIES} )

#add_executable(try_stm32_pc src/try_stm32_pc.cpp)
#add_dependencies(try_stm32_pc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(try_stm32_pc ${catkin_LIBRARIES} )
