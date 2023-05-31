cmake_minimum_required(VERSION 3.0.2)
project(repair_urdf)

find_package(catkin REQUIRED) 
find_package(xacro REQUIRED)

catkin_package()

# Xacro files
file(GLOB xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/urdf/repair_full.urdf.xacro)

foreach(it ${xacro_files})
  # remove .xacro extension
  string(REGEX MATCH "(.*)[.]xacro$" unused ${it})
  set(output_filename ${CMAKE_MATCH_1})

  # create a rule to generate ${output_filename} from {it}
  xacro_add_xacro_file(${it} ${output_filename})

  list(APPEND URDF_files ${output_filename})
endforeach(it)

# add an abstract target to actually trigger the builds
add_custom_target(urdf_files ALL DEPENDS ${URDF_files})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
