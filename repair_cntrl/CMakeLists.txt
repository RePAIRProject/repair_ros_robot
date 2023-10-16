cmake_minimum_required(VERSION 3.1.3)
project(repair_cntrl)

find_package(catkin REQUIRED roscpp rospy urdf sensor_msgs std_msgs message_generation)
find_package(XBotInterface REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(matlogger2 REQUIRED)
find_package(cartesian_interface REQUIRED)
find_package(xbot2 QUIET)

option(XBOT2_ENABLE_XENO OFF "Compile against xenomai") 


generate_messages(
  DEPENDENCIES
  std_msgs
)

install(DIRECTORY
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_package(CATKIN_DEPENDS message_runtime)

