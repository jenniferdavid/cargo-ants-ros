cmake_minimum_required(VERSION 2.8.3)
project(at_udp2ros)

#set C++11 support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Build type
IF (NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE "RELEASE") 
ENDIF (NOT CMAKE_BUILD_TYPE)

## Find catkin macros and libraries
find_package( catkin REQUIRED COMPONENTS 
      roscpp
      rospy
      std_msgs
      nav_msgs
      sensor_msgs
      visualization_msgs
      tf
)

##SRCS files.
SET(SRCS
      src/ros_publisher.cpp
      src/udp_receiver.cpp
)

## The catkin_package macro generates cmake config files for your package
catkin_package (
      CATKIN_DEPENDS 
            roscpp 
            rospy 
            std_msgs 
            nav_msgs
            sensor_msgs
            visualization_msgs
            tf
)

# add to include path
include_directories(${catkin_INCLUDE_DIRS})

# build executable
add_executable(at_udp2ros src/udp2ros_main.cpp ${SRCS})
target_link_libraries(at_udp2ros ${catkin_LIBRARIES})
