# Install script for directory: /home/cargoants/Documents/poftwaresatent_hhcoaching/hhcoaching/Jenni/catkin_ws/src/laser_scan_utils/src

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "RELEASE")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri-algorithms/liblaser_scan_utils.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri-algorithms/liblaser_scan_utils.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri-algorithms/liblaser_scan_utils.so"
         RPATH "")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/iri-algorithms" TYPE SHARED_LIBRARY FILES "/home/cargoants/Documents/poftwaresatent_hhcoaching/hhcoaching/Jenni/catkin_ws/src/laser_scan_utils/lib/liblaser_scan_utils.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri-algorithms/liblaser_scan_utils.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri-algorithms/liblaser_scan_utils.so")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri-algorithms/liblaser_scan_utils.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iri-algorithms/laser_scan_utils" TYPE FILE FILES "/home/cargoants/Documents/poftwaresatent_hhcoaching/hhcoaching/Jenni/catkin_ws/src/laser_scan_utils/src/laser_scan_utils.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iri-algorithms/laser_scan_utils" TYPE FILE FILES
    "/home/cargoants/Documents/poftwaresatent_hhcoaching/hhcoaching/Jenni/catkin_ws/src/laser_scan_utils/src/corner_detector.h"
    "/home/cargoants/Documents/poftwaresatent_hhcoaching/hhcoaching/Jenni/catkin_ws/src/laser_scan_utils/src/entities.h"
    "/home/cargoants/Documents/poftwaresatent_hhcoaching/hhcoaching/Jenni/catkin_ws/src/laser_scan_utils/src/laser_scan.h"
    "/home/cargoants/Documents/poftwaresatent_hhcoaching/hhcoaching/Jenni/catkin_ws/src/laser_scan_utils/src/line_detector.h"
    "/home/cargoants/Documents/poftwaresatent_hhcoaching/hhcoaching/Jenni/catkin_ws/src/laser_scan_utils/src/line_finder.h"
    "/home/cargoants/Documents/poftwaresatent_hhcoaching/hhcoaching/Jenni/catkin_ws/src/laser_scan_utils/src/line_finder_hough.h"
    "/home/cargoants/Documents/poftwaresatent_hhcoaching/hhcoaching/Jenni/catkin_ws/src/laser_scan_utils/src/line_finder_jump_fit.h"
    "/home/cargoants/Documents/poftwaresatent_hhcoaching/hhcoaching/Jenni/catkin_ws/src/laser_scan_utils/src/line_segment.h"
    "/home/cargoants/Documents/poftwaresatent_hhcoaching/hhcoaching/Jenni/catkin_ws/src/laser_scan_utils/src/scan_basics.h"
    "/home/cargoants/Documents/poftwaresatent_hhcoaching/hhcoaching/Jenni/catkin_ws/src/laser_scan_utils/src/scan_segment.h"
    "/home/cargoants/Documents/poftwaresatent_hhcoaching/hhcoaching/Jenni/catkin_ws/src/laser_scan_utils/src/clustering.h"
    "/home/cargoants/Documents/poftwaresatent_hhcoaching/hhcoaching/Jenni/catkin_ws/src/laser_scan_utils/src/object_detector.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/share/cmake-2.8/Modules/Findlaser_scan_utils.cmake")
  IF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
  IF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
FILE(INSTALL DESTINATION "/usr/share/cmake-2.8/Modules" TYPE FILE FILES "/home/cargoants/Documents/poftwaresatent_hhcoaching/hhcoaching/Jenni/catkin_ws/src/laser_scan_utils/src/../Findlaser_scan_utils.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/cargoants/Documents/poftwaresatent_hhcoaching/hhcoaching/Jenni/catkin_ws/src/laser_scan_utils/build/src/examples/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

