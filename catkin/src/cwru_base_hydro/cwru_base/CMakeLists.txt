cmake_minimum_required(VERSION 2.8.3)
project(cwru_base)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  #sicktoolbox_wrapper
  std_msgs
  cwru_msgs
  std_srvs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp geometry_msgs 
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  INCLUDE_DIRS include 
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(cwru_base
#   src/${PROJECT_NAME}/cwru_base.cpp
# )

## Declare a cpp executable
add_executable(crio_receiver src/crio_receiver.cpp)

# THIS LINE ADDED TO FIX DEPENDENCY ISSUE FROM CWRU_MSGS
add_dependencies(crio_receiver ${catkin_EXPORTED_TARGETS})

target_link_libraries(crio_receiver ${catkin_LIBRARIES})

