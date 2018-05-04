cmake_minimum_required(VERSION 2.8.3)

project(hardware)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  #std_msgs
  serial_arm_lib
  ipab_lwr_msgs
  urdf
  kdl_parser
  #roslib
  #orocos_kdl
)

#find_package(orocos_kdl)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kuka gamma cab optoforce
  CATKIN_DEPENDS roscpp message_runtime ipab_lwr_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

# Define a library target called kuka
add_library(kuka src/kuka.cpp)
target_link_libraries(kuka ${catkin_LIBRARIES})

# Define a library target called cab
add_library(cab src/cab.cpp)
target_link_libraries(cab ${catkin_LIBRARIES})

# Define a library target called gamma
add_library(gamma src/gamma.cpp)
target_link_libraries(gamma ${catkin_LIBRARIES})

# Define a library target called optoforce
add_library(optoforce src/optoforce.cpp)
target_link_libraries(optoforce ${catkin_LIBRARIES})
