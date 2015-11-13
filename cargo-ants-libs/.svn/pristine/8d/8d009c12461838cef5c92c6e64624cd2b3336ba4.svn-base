cmake_minimum_required (VERSION 2.6)
if (COMMAND cmake_policy)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
  if (POLICY CMP0011)
    cmake_policy (SET CMP0011 NEW)
  endif (POLICY CMP0011)
endif (COMMAND cmake_policy)

##################################################
# basic setup

project (FPPLIB)
set (CMAKE_VERBOSE_MAKEFILE ON)
if (NOT BUILD_SHARED_LIBS)
  set (BUILD_SHARED_LIBS True)
endif (NOT BUILD_SHARED_LIBS)
if (NOT CMAKE_BUILD_TYPE)
  SET (CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

include_directories (
  include
  yaml-cpp-0.2.7/include
  )

##################################################
# OS detection

if (WIN32)
  add_definitions (-DWIN32)
else (WIN32)
  if (APPLE)
    add_definitions (-DOSX)
    include_directories (
      /opt/local/include
      )
    list (APPEND CMAKE_REQUIRED_INCLUDES
      /opt/local/include
      )
    link_directories (
      /opt/local/lib
      )
  endif (APPLE)
  if (CMAKE_SYSTEM_NAME MATCHES Linux)
    add_definitions (-DLINUX)
  endif (CMAKE_SYSTEM_NAME MATCHES Linux)
  if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    add_definitions (-DOPENBSD)
    include_directories (
      /usr/local/include
      )
    link_directories (
      /usr/local/lib
      )
  endif (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
endif(WIN32)

##################################################
# compiler flags (other than header and library paths)

include (CheckCXXCompilerFlag)

check_cxx_compiler_flag (-pipe CXX_FLAG_pipe)
if (CXX_FLAG_pipe)
  add_definitions (-pipe)
endif (CXX_FLAG_pipe)
check_cxx_compiler_flag (-Wall CXX_FLAG_Wall)
if (CXX_FLAG_Wall)
  add_definitions (-Wall)
endif (CXX_FLAG_Wall)

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  check_cxx_compiler_flag (-O0 CXX_FLAG_O0)
  if (CXX_FLAG_O0)
    add_definitions (-O0)
  endif (CXX_FLAG_O0)
endif (${CMAKE_BUILD_TYPE} STREQUAL "Debug")

##################################################
### Build 3rdparty stuff

subdirs (
  yaml-cpp-0.2.7
  )

##################################################
### The actual sources

list (APPEND FPPLIB_SRCS
  src/factory.cpp
  src/configurable.cpp
  src/parameter.cpp
  src/yaml_parser.cpp
  )

add_library (fpp SHARED ${FPPLIB_SRCS})
target_link_libraries (fpp yaml-cpp)

add_executable (test test/test.cpp)
target_link_libraries (test fpp)

add_executable (test-npm test/test-npm.cpp)
target_link_libraries (test-npm fpp)

add_executable (idea test/idea.cpp)
target_link_libraries (idea fpp)

add_executable (test-callback test/test-callback.cpp)
target_link_libraries (test-callback fpp)

add_executable (tut01-basic-parameters test/tut01-basic-parameters.cpp)
target_link_libraries (tut01-basic-parameters fpp)
