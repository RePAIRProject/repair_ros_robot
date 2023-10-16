cmake_minimum_required(VERSION 3.0.2)
project(repair_srdf)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
