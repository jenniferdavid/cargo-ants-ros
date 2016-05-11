cmake_minimum_required (VERSION 2.6)
if (COMMAND cmake_policy)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
  if (POLICY CMP0011)
    cmake_policy (SET CMP0011 NEW)
  endif (POLICY CMP0011)
  if (POLICY CMP0042)
    cmake_policy (SET CMP0042 NEW)
  endif (POLICY CMP0042)
endif (COMMAND cmake_policy)

project (cargo_ants_rol)
set (CMAKE_VERBOSE_MAKEFILE ON)

find_package (catkin REQUIRED COMPONENTS
  cargo_ants_msgs
  roscpp
)

catkin_package (
  INCLUDE_DIRS include
  LIBRARIES cargo_ants_rol
  CATKIN_DEPENDS cargo_ants_msgs roscpp
  #  DEPENDS system_lib
)

include_directories (
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library (cargo_ants_rol SHARED
  src/rolMessage.cpp
  src/rolClient.cpp
  src/rolServer.cpp
  )
#target_link_libraries ()

add_executable (test_rol src/test_rol.cpp)
target_link_libraries (test_rol cargo_ants_rol)
add_dependencies(test_rol ${catkin_EXPORTED_TARGETS})

add_executable (test_feed_rol2ros src/test_feed_rol2ros.cpp)
target_link_libraries (test_feed_rol2ros cargo_ants_rol)
add_dependencies(test_feed_rol2ros ${catkin_EXPORTED_TARGETS})

add_executable (test_node_rol2ros src/test_node_rol2ros.cpp)
target_link_libraries (test_node_rol2ros cargo_ants_rol ${catkin_LIBRARIES})
add_dependencies(test_node_rol2ros ${catkin_EXPORTED_TARGETS})

add_executable (test_node_ros2rol src/test_node_ros2rol.cpp)
target_link_libraries (test_node_ros2rol cargo_ants_rol ${catkin_LIBRARIES})
add_dependencies(test_node_ros2rol ${catkin_EXPORTED_TARGETS})
