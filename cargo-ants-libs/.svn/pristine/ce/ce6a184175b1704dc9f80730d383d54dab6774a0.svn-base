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

##################################################
# basic setup

project (SFL2)
set (CMAKE_VERBOSE_MAKEFILE ON)
if (NOT BUILD_SHARED_LIBS)
  set (BUILD_SHARED_LIBS True)
endif (NOT BUILD_SHARED_LIBS)
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

if (CMAKE_BUILD_TYPE STREQUAL Debug)
  add_definitions (-DSFL2_DEBUG)
endif (CMAKE_BUILD_TYPE STREQUAL Debug)

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
# configure-time checks

include (CheckIncludeFileCXX)

# find Boost (smart pointers and bind)

if (BOOST_DIR)
  list (APPEND CMAKE_REQUIRED_INCLUDES
    ${BOOST_DIR}/include
    ${BOOST_DIR}
    ${BOOST_DIR}/include/boost
    ${BOOST_DIR}/boost
    )
  include_directories (
    ${BOOST_DIR}/include
    ${BOOST_DIR}
    ${BOOST_DIR}/include/boost
    ${BOOST_DIR}/boost
    )
endif (BOOST_DIR)
check_include_file_cxx (boost/shared_ptr.hpp HAVE_BOOST)
if (HAVE_BOOST)
  message ("*** found Boost (required)")
else (HAVE_BOOST)
  message (FATAL_ERROR "Please install the Boost smart_ptr library, e.g. `sudo apt-get install libboost-dev' and/or tell me where to find it by passing -DBOOST_DIR:path=/path/to/boost to cmake next time (you probably need to clear its cache first).")
endif (HAVE_BOOST)

include (FindOpenGL)
if (OPENGL_FOUND)
  message ("*** found OpenGL (required)")
else (OPENGL_FOUND)
  message (FATAL_ERROR "Please install OpenGL.")
endif (OPENGL_FOUND)
if (OPENGL_GLU_FOUND)
  message ("*** found GLU (required)")
else (OPENGL_GLU_FOUND)
  message (FATAL_ERROR "Please install GLU.")
endif (OPENGL_GLU_FOUND)

include (FindGLUT)
if (GLUT_FOUND)
  message ("*** found GLUT (required)")
else (GLUT_FOUND)
  message (FATAL_ERROR "Please install GLUT.")
endif (GLUT_FOUND)

if (ESTAR_DIR)
  list (APPEND CMAKE_REQUIRED_INCLUDES
    ${ESTAR_DIR}/include
    ${ESTAR_DIR}
    )
  include_directories (
    ${ESTAR_DIR}/include
    ${ESTAR_DIR}
    )
  link_directories (
    ${ESTAR_DIR}/lib
    ${ESTAR_DIR}
    )
endif (ESTAR_DIR)
check_include_file_cxx (estar/Facade.hpp HAVE_ESTAR)
if (HAVE_ESTAR)
  message ("*** found Estar (optional)")
  add_definitions (-DSFL2_HAVE_ESTAR)
endif (HAVE_ESTAR)

##################################################
### The actual project

if (NOT NPM_PLUGIN_PATH)
  message ("setting NPM_PLUGIN_PATH to ${SFL2_BINARY_DIR}/plugins")
  set (NPM_PLUGIN_PATH ${SFL2_BINARY_DIR}/plugins)
else (NOT NPM_PLUGIN_PATH)
  message ("NPM_PLUGIN_PATH is ${NPM_PLUGIN_PATH}")
endif (NOT NPM_PLUGIN_PATH)
add_definitions (-DNPM_PLUGIN_PATH_STR="${NPM_PLUGIN_PATH}")

include_directories (
  .
  3rdparty/fpplib/include
  3rdparty/fpplib/yaml-cpp-0.2.7/include
  ${OPENGL_INCLUDE_DIR}
  ${GLUT_INCLUDE_DIR}
  )

file (GLOB         SFL2_YAMLCPP_SRCS  "3rdparty/fpplib/yaml-cpp-0.2.7/src/*.cpp")
file (GLOB         SFL2_FPPLIB_SRCS   "3rdparty/fpplib/src/*.cpp")
file (GLOB_RECURSE SFL2_SFL_SRCS      "sfl/*.cpp")
file (GLOB_RECURSE SFL2_NPM_SRCS      "npm/*.cpp")
file (GLOB_RECURSE SFL2_NPMESTAR_SRCS "opt/npmestar/*.cpp")

add_library (yamlcpp SHARED ${SFL2_YAMLCPP_SRCS})
add_library (fpplib  SHARED ${SFL2_FPPLIB_SRCS})
target_link_libraries (fpplib yamlcpp)
add_library (sfl     SHARED ${SFL2_SFL_SRCS})

if (HAVE_ESTAR)
  add_library (npm SHARED ${SFL2_NPM_SRCS} ${SFL2_NPMESTAR_SRCS})
  target_link_libraries (npm fpplib sfl estar ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
else (HAVE_ESTAR)
  add_library (npm SHARED ${SFL2_NPM_SRCS})
  target_link_libraries (npm fpplib sfl ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
endif (HAVE_ESTAR)

add_executable (nepumuk apps/nepumuk.cpp)
target_link_libraries (nepumuk npm)

add_executable (test-plugin apps/test-plugin.cpp)
target_link_libraries (test-plugin npm)

add_library (plugin-test MODULE apps/plugin-test.cpp)
target_link_libraries (plugin-test npm) # not sure if this is required on Linux
set_target_properties (plugin-test PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${NPM_PLUGIN_PATH})

##################################################
# Nepumuk v2

add_definitions (-DNPM2PATH_STR="${SFL2_BINARY_DIR}")

add_library (npm2 SHARED
  npm2/Body.cpp
  npm2/BBox.cpp
  npm2/Object.cpp
  npm2/Actuator.cpp
  npm2/Sensor.cpp
  npm2/GenericDrive.cpp
  npm2/DifferentialDrive.cpp
  npm2/DifferentialTrailerDrive.cpp
  npm2/RevoluteServo.cpp
  npm2/RayDistanceSensor.cpp
  npm2/Simulator.cpp
  npm2/Factory.cpp
  npm2/gl.cpp
  npm2/Process.cpp
  npm2/KinematicControl.cpp
  npm2/Viewport.cpp
  npm2/View.cpp
  npm2/Camera.cpp
  npm2/Drawing.cpp
  npm2/ObjectCamera.cpp
  npm2/ObjectDrawing.cpp
  npm2/Plugin.cpp
  npm2/FixedCamera.cpp
  )
target_link_libraries (npm2 sfl fpplib ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

add_library (abcd-plugin MODULE npm2/abcd-plugin.cpp)
target_link_libraries (abcd-plugin npm2)

add_library (pickplace-plugin MODULE npm2/pickplace-plugin.cpp)
target_link_libraries (pickplace-plugin npm2)

add_library (containerteleport-plugin MODULE npm2/containerteleport-plugin.cpp)
target_link_libraries (containerteleport-plugin npm2)

add_executable (nepumuk2 npm2/nepumuk2.cpp)
target_link_libraries (nepumuk2 npm2)
