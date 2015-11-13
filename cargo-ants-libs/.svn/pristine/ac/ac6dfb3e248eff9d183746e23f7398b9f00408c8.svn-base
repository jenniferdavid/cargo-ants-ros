cmake_minimum_required(VERSION 2.8.3)
project(sfl2)

set (CMAKE_VERBOSE_MAKEFILE ON)

##################################################
# OS detection (needed for GL support)

if (WIN32)
  add_definitions (-DWIN32)
else (WIN32)
  if (APPLE)
    add_definitions (-DOSX)
  endif (APPLE)
  if (CMAKE_SYSTEM_NAME MATCHES Linux)
    add_definitions (-DLINUX)
  endif (CMAKE_SYSTEM_NAME MATCHES Linux)
  if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    add_definitions (-DOPENBSD)
  endif (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
endif(WIN32)

##################################################
# ROS stuff

find_package(catkin REQUIRED COMPONENTS
  roscpp
#  message_generation
)

#add_message_files(
#  DIRECTORY msg
#  FILES
#  Scan.msg
#)
#
#generate_messages()

catkin_package(
  INCLUDE_DIRS
  ..
  ../3rdparty/fpplib/include
  ../3rdparty/fpplib/yaml-cpp-0.2.7/include
  LIBRARIES npm
  CATKIN_DEPENDS roscpp
  #  DEPENDS system_lib
)

set (NPM_PLUGIN_PATH ${sfl2_BINARY_DIR}/plugins)
subdirs (..)

include_directories(
  ..
  ../3rdparty/fpplib/include
  ../3rdparty/fpplib/yaml-cpp-0.2.7/include
  ${catkin_INCLUDE_DIRS}
)
