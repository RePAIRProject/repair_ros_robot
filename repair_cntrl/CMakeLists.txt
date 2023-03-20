cmake_minimum_required(VERSION 3.1.3)
project(repair_cntrl)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES bin/libxbot_rob_mon_plugin_softhand.so DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


