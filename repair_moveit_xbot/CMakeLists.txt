cmake_minimum_required(VERSION 3.0.2)
project(repair_moveit_xbot)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-fpermissive")
add_compile_options(
  -O3
)

set (CMAKE_CXX_FLAGS "-g")

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  control_msgs
  std_msgs
  tf2_ros
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rviz_visual_tools
  moveit_visual_tools
  interactive_markers
  message_generation
  xbot_msgs
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  control_msgs
  xbot_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES repair_interface
 CATKIN_DEPENDS dynamic_reconfigure geometry_msgs roscpp rospy sensor_msgs std_msgs message_runtime
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

add_executable(moveit_xbot_bridge_node
  src/moveit_xbot_bridge.cpp
)

add_dependencies(moveit_xbot_bridge_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(moveit_xbot_bridge_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY
  config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
