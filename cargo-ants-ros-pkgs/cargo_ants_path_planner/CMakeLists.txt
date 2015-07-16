cmake_minimum_required(VERSION 2.8.3)
project(cargo_ants_path_planner)

set (CMAKE_VERBOSE_MAKEFILE ON)
set (CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  cargo_ants_msgs
  cargo_ants_util
  estar2
  sfl2
  roscpp
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
  #  LIBRARIES cargo_ants_path_planner
  CATKIN_DEPENDS cargo_ants_msgs roscpp
  #  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(path_planner src/path_planner.cpp)
target_link_libraries(path_planner ${catkin_LIBRARIES})
