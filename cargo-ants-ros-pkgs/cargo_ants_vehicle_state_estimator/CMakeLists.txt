cmake_minimum_required(VERSION 2.8.3)
project(cargo_ants_vehicle_state_estimator)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs)

## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS roscpp rospy geometry_msgs)

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(vehicle_state_estimator_node src/vehicle_state_estimator_main.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(vehicle_state_estimator_node ${PROJECT_NAME}_EXPORTED_TARGETS)

## Specify libraries to link a library or executable target against
target_link_libraries(vehicle_state_estimator_node ${catkin_LIBRARIES})


