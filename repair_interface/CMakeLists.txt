cmake_minimum_required(VERSION 3.12)
project(repair_interface)

set(CMAKE_CXX_STANDARD 17)
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
  trajectory_msgs
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
  qb_device_srvs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetCurrentPose.srv
  MoveArmToPose.srv
  MoveBothArms.srv
  GripperCommand.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  control_msgs
  xbot_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES repair_interface
 CATKIN_DEPENDS dynamic_reconfigure geometry_msgs roscpp rospy sensor_msgs std_msgs message_runtime qb_device_srvs
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

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/moveit_client.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Declare C++ executable
add_executable(${PROJECT_NAME}_node
  src/repair_interface_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(dawnik_bridge_node
  src/dawnik_bridge_node.cpp
)

add_dependencies(dawnik_bridge_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(dawnik_bridge_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
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

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
