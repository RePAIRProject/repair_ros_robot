cmake_minimum_required(VERSION 3.0.2)
project(repair_srdf)

find_package(catkin REQUIRED)
find_package(xacro REQUIRED)

catkin_package()

# Xacro files
file(GLOB xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/srdf/*.xacro)

foreach(it ${xacro_files})
  # remove .xacro extension
  string(REGEX MATCH "(.*)[.]xacro$" unused ${it})
  set(output_filename ${CMAKE_MATCH_1})

  # create a rule to generate ${output_filename} from {it}
  xacro_add_xacro_file(${it} ${output_filename})

  list(APPEND SRDF_files ${output_filename})
endforeach(it)

# add an abstract target to actually trigger the builds
add_custom_target(srdf_files ALL DEPENDS ${SRDF_files})

install(DIRECTORY
  srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
