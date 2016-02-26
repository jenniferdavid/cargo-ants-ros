#edit the following line to add the librarie's header files
FIND_PATH(
    laser_scan_utils_INCLUDE_DIRS
    NAMES laser_scan_utils.h
    PATHS /usr/local/include/iri-algorithms/laser_scan_utils)
#change INCLUDE_DIRS to its parent directory
get_filename_component(laser_scan_utils_INCLUDE_DIRS ${laser_scan_utils_INCLUDE_DIRS} DIRECTORY)
MESSAGE(STATUS "laser_scan_utils_INCLUDE_DIRS: ${laser_scan_utils_INCLUDE_DIRS}")

FIND_LIBRARY(
    laser_scan_utils_LIBRARY
    NAMES laser_scan_utils
    PATHS /usr/lib /usr/local/lib /usr/local/lib/iri-algorithms) 
    
MESSAGE(STATUS "laser_scan_utils_LIBRARY: ${laser_scan_utils_LIBRARY}")

IF (laser_scan_utils_INCLUDE_DIRS AND laser_scan_utils_LIBRARY)
   SET(laser_scan_utils_FOUND TRUE)
ENDIF (laser_scan_utils_INCLUDE_DIRS AND laser_scan_utils_LIBRARY)

IF (laser_scan_utils_FOUND)
   IF (NOT laser_scan_utils_FIND_QUIETLY)
      MESSAGE(STATUS "Found laser_scan_utils: ${laser_scan_utils_LIBRARY}")
   ENDIF (NOT laser_scan_utils_FIND_QUIETLY)
ELSE (laser_scan_utils_FOUND)
   IF (laser_scan_utils_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find laser_scan_utils")
   ENDIF (laser_scan_utils_FIND_REQUIRED)
ENDIF (laser_scan_utils_FOUND)

