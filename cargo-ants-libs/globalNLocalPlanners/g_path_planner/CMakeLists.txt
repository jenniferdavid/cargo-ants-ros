cmake_minimum_required(VERSION 2.8.3)
project(g_path_planner)

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

#Add eigen3 includes. Required when eigen *.h files call other eigen *.h file. TODO: generalize that to machine though some "find" macro
INCLUDE_DIRECTORIES("/usr/include/eigen3")

## Find catkin macros and libraries
find_package( catkin REQUIRED COMPONENTS 
    roscpp
    nav_core
    rospy
    std_msgs
    nav_msgs
    sensor_msgs
    tf
    genmsg 
    cargo_ants_msgs)
    
 
    
#SRCS files. ROS node
SET(SRCS_ROS_NODE 
	  src/cluster.cpp
      src/grid_2d.cpp
      src/local_map_node.cpp
)


## The catkin_package macro generates cmake config files for your package
catkin_package ( CATKIN_DEPENDS 
        roscpp 
        rospy 
        std_msgs 
        nav_msgs
        sensor_msgs
        tf 
        cargo_ants_msgs 
        nav_core roscpp rospy std_msgs
        )

# add to include path
include_directories(
  ${catkin_INCLUDE_DIRS}
)
 
add_library(relaxed_astar_lib src/RAstar_ros.cpp)

add_executable(MakePlan src/MakePlan.cpp)
target_link_libraries(MakePlan ${catkin_LIBRARIES})
