cmake_minimum_required(VERSION 2.8.3)
project(cargo_ants_util)

find_package (catkin REQUIRED
  COMPONENTS
  cargo_ants_msgs
  sfl2
  estar2
  roscpp
  )
find_package (cmake_modules REQUIRED)
find_package (Eigen REQUIRED)

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
  INCLUDE_DIRS include ${EIGEN_INCLUDE_DIRS}
  LIBRARIES cargo_ants_util
  CATKIN_DEPENDS
  cargo_ants_msgs
  #eigen
  sfl2
  estar2
  roscpp
  #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_library (cargo_ants_util
  src/util.cpp
  src/estar_util.cpp
  )
target_link_libraries (cargo_ants_util ${catkin_LIBRARIES})
