cmake_minimum_required(VERSION 2.8.3)
project(cargo_ants_task_scheduler)

find_package(catkin REQUIRED COMPONENTS
  cargo_ants_msgs
  sfl2
  estar2
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
  INCLUDE_DIRS include
  LIBRARIES task_scheduler_lib
  CATKIN_DEPENDS
  cargo_ants_msgs
  sfl2
  estar2
  roscpp
  #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(task_scheduler src/task_scheduler.cpp)
target_link_libraries(task_scheduler ${catkin_LIBRARIES} sfl estar2)

add_executable(draft_scheduler src/draft_scheduler.cpp)
target_link_libraries(draft_scheduler ${catkin_LIBRARIES})
