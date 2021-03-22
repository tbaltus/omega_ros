cmake_minimum_required(VERSION 3.0.2)
project(omega_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES omega_ros
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_library(DHDLIB libdhd.so lib)
find_library(DRDLIB libdrd.so lib)
find_library(USB1LIB  libusb-1.0.so)

add_executable(omega7 
  src/omega7.cpp)

add_dependencies(omega7 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(omega7
  ${DHDLIB}
  ${DRDLIB}
  ${USB1LIB}
  ${catkin_LIBRARIES})

add_executable(hello_world 
  src/hello_world.cpp)

add_dependencies(hello_world ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hello_world
  ${DHDLIB}
  ${DRDLIB}
  ${USB1LIB}
  ${catkin_LIBRARIES})