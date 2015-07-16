cmake_minimum_required(VERSION 2.8.3)
project(cargo_ants_npm)

set (CMAKE_VERBOSE_MAKEFILE ON)
set (CMAKE_BUILD_TYPE Debug)

if (APPLE)
  add_definitions (-DOSX)
endif (APPLE)
if (CMAKE_SYSTEM_NAME MATCHES Linux)
  add_definitions (-DLINUX)
endif (CMAKE_SYSTEM_NAME MATCHES Linux)

find_package(catkin REQUIRED COMPONENTS
  cargo_ants_msgs
  sfl2
  roscpp
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  TraversabilityMap.msg
  Estar.msg
)

generate_messages(
  DEPENDENCIES
  cargo_ants_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #  INCLUDE_DIRS include
###  LIBRARIES sfl2 npm
  CATKIN_DEPENDS sfl2 cargo_ants_msgs roscpp
  #  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_library (cargo-ants-plugin MODULE src/cargo-ants-plugin.cpp)
target_link_libraries (cargo-ants-plugin npm ${catkin_LIBRARIES})
#set_target_properties (cargo-ants-plugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${NPM_PLUGIN_PATH})
