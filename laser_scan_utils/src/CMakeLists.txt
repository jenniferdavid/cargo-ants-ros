
MESSAGE("Starting laser_scan_utils CMakeLists ...")
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#find dependencies. 
FIND_PACKAGE(faramotics QUIET) #faramotics is only required for some tests
IF(faramotics_FOUND)
	#FIND_PACKAGE(GLUT REQUIRED)
	FIND_PACKAGE(pose_state_time REQUIRED)
    MESSAGE("Faramotics Library FOUND: Tests requiring it will be built.")
ENDIF(faramotics_FOUND)

#include directories
INCLUDE_DIRECTORIES(.)
IF(Ceres_FOUND)
    INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
ENDIF(Ceres_FOUND)
IF(faramotics_FOUND)
    INCLUDE_DIRECTORIES(${faramotics_INCLUDE_DIRS})
ENDIF(faramotics_FOUND)

#headers
SET(HDRS_BASE
    laser_scan_utils.h)
    
SET(HDRS
    corner_detector.h
    entities.h
    laser_scan.h
    line_detector.h
    line_finder.h
    line_finder_hough.h
    line_finder_jump_fit.h
    line_segment.h
    scan_basics.h
    scan_segment.h
    clustering.h
    object_detector.h)

#sources
SET(SRCS
    corner_detector.cpp
    entities.cpp
    laser_scan.cpp
    line_detector.cpp
    line_finder.cpp
    line_finder_hough.cpp
    line_finder_jump_fit.cpp
    line_segment.cpp
    scan_basics.cpp
    scan_segment.cpp
    clustering.cpp
    object_detector.cpp)
    
# create the shared library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SRCS})

#install library
INSTALL(TARGETS ${PROJECT_NAME}
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib/iri-algorithms
      ARCHIVE DESTINATION lib/iri-algorithms)
        
#install headers         
INSTALL(FILES ${HDRS_BASE} 
      DESTINATION include/iri-algorithms/laser_scan_utils)
INSTALL(FILES ${HDRS} 
      DESTINATION include/iri-algorithms/laser_scan_utils)

#install Find*.cmake
INSTALL(FILES ../Findlaser_scan_utils.cmake DESTINATION ${CMAKE_ROOT}/Modules/)

#Build examples & tests
MESSAGE("Building examples and tests.")
ADD_SUBDIRECTORY(examples)
