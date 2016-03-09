cmake_minimum_required(VERSION 2.8.3)
project(wolf_ros)

#Set compiler according C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++0x support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED)
# ******************************************************************** 
#                 Add catkin additional components here
# ******************************************************************** 
find_package(catkin REQUIRED COMPONENTS iri_base_algorithm visualization_msgs nav_msgs sensor_msgs tf)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# ******************************************************************** 
#           Add system and labrobotica dependencies here
# ******************************************************************** 
find_package(Ceres REQUIRED)
find_package(laser_scan_utils REQUIRED)
find_package(wolf REQUIRED)
find_package(raw_gps_utils QUIET) #raw_gps_utils is not required
IF(raw_gps_utils_FOUND)
        MESSAGE("raw_gps_utils Library FOUND: raw_gps_utils related sources will be built.")
ENDIF(raw_gps_utils_FOUND)


# ******************************************************************** 
#           Add topic, service and action definition here
# ******************************************************************** 
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

# ******************************************************************** 
#                 Add the dynamic reconfigure file 
# ******************************************************************** 
generate_dynamic_reconfigure_options(cfg/Wolf.cfg)

# ******************************************************************** 
#                 Add run time dependencies here
# ******************************************************************** 
catkin_package(
#  INCLUDE_DIRS 
#  LIBRARIES 
# ******************************************************************** 
#            Add ROS and IRI ROS run time dependencies
# ******************************************************************** 
 CATKIN_DEPENDS iri_base_algorithm visualization_msgs nav_msgs sensor_msgs
# ******************************************************************** 
#      Add system and labrobotica run time dependencies here
# ******************************************************************** 
  DEPENDS Eigen
)

###########
## Build ##
###########

# ******************************************************************** 
#                   Add the include directories 
# ******************************************************************** 
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${CERES_INCLUDE_DIRS}
                    ${laser_scan_utils_INCLUDE_DIRS}
                    ${wolf_INCLUDE_DIRS})
message("wolf_INCLUDE_DIRS: ${wolf_INCLUDE_DIRS}")
message("laser_scan_utils_INCLUDE_DIRS: ${laser_scan_utils_INCLUDE_DIRS}")
IF(raw_gps_utils_FOUND)
        include_directories(${raw_gps_utils_INCLUDE_DIRS})
        message("raw_gps_utils_INCLUDE_DIRS: ${raw_gps_utils_INCLUDE_DIRS}")
ENDIF(raw_gps_utils_FOUND)
                    
## Declare a cpp library
# add_library(${PROJECT_NAME} <list of source files>)

## Declare a cpp executable
add_executable(${PROJECT_NAME} src/wolf_alg.cpp src/wolf_alg_node.cpp)
add_executable(wolf_prunning src/wolf_prunning_test.cpp)
IF(raw_gps_utils_FOUND)
        add_executable(wolf_gps_node src/wolf_gps_main.cpp src/wolf_gps_node.cpp)
ENDIF(raw_gps_utils_FOUND)
# ******************************************************************** 
#                   Add the libraries
# ******************************************************************** 
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${Ceres_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${laser_scan_utils_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${wolf_LIBRARY})
target_link_libraries(wolf_prunning ${catkin_LIBRARIES})
target_link_libraries(wolf_prunning ${CERES_LIBRARIES})
target_link_libraries(wolf_prunning ${wolf_LIBRARY})
IF(raw_gps_utils_FOUND)
        target_link_libraries(wolf_gps_node ${catkin_LIBRARIES})
        target_link_libraries(wolf_gps_node ${CERES_LIBRARIES})
        target_link_libraries(wolf_gps_node ${wolf_LIBRARY})
        target_link_libraries(wolf_gps_node ${raw_gps_utils_LIBRARY})
ENDIF(raw_gps_utils_FOUND)

# ******************************************************************** 
#               Add message headers dependencies 
# ******************************************************************** 
# add_dependencies(${PROJECT_NAME} <msg_package_name>_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} visualization_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} nav_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} sensor_msgs_generate_messages_cpp)
IF(raw_gps_utils_FOUND)
        add_dependencies(wolf_gps_node iri_common_drivers_msgs_generate_messages_cpp)
ENDIF(raw_gps_utils_FOUND)
# ******************************************************************** 
#               Add dynamic reconfigure dependencies 
# ******************************************************************** 
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
