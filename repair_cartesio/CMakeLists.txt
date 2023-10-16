cmake_minimum_required(VERSION 3.0.2)
project(repair_cartesio)

find_package(catkin REQUIRED) 

catkin_package()

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)