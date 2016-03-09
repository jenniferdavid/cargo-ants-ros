cmake_minimum_required(VERSION 2.8.3)
project(cargo_ants_slam)

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
# find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
        iri_base_algorithm 
        visualization_msgs 
        nav_msgs 
        sensor_msgs 
        tf)

# System dependencies
find_package(Ceres REQUIRED)
find_package(laser_scan_utils REQUIRED)
find_package(wolf REQUIRED)

# Add the dynamic reconfigure file 
generate_dynamic_reconfigure_options(cfg/cargo_ants_slam.cfg)

# Add run time dependencies here
catkin_package(
    CATKIN_DEPENDS 
        iri_base_algorithm 
        visualization_msgs 
        nav_msgs 
        sensor_msgs
    DEPENDS 
        Eigen
)

# Add the include directories 
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${laser_scan_utils_INCLUDE_DIRS}
        ${wolf_INCLUDE_DIRS})
# message("wolf_INCLUDE_DIRS: ${wolf_INCLUDE_DIRS}")
# message("laser_scan_utils_INCLUDE_DIRS: ${laser_scan_utils_INCLUDE_DIRS}")

## Declare cpp executables & Link them with the libraries
add_executable(wolf_node src/wolf_alg.cpp src/wolf_alg_node.cpp)
target_link_libraries(wolf_node ${catkin_LIBRARIES})
target_link_libraries(wolf_node ${Ceres_LIBRARIES})
target_link_libraries(wolf_node ${laser_scan_utils_LIBRARY})
target_link_libraries(wolf_node ${wolf_LIBRARY})

add_executable(${PROJECT_NAME}_node src/slam_main.cpp src/slam_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${Ceres_LIBRARIES})
#target_link_libraries(${PROJECT_NAME}_node ${laser_scan_utils_LIBRARY})
target_link_libraries(${PROJECT_NAME}_node ${wolf_LIBRARY})

# Add message headers dependencies 
# add_dependencies(wolf_node visualization_msgs_generate_messages_cpp)
# add_dependencies(wolf_node nav_msgs_generate_messages_cpp)
# add_dependencies(wolf_node sensor_msgs_generate_messages_cpp)

# Add dynamic reconfigure dependencies 
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
