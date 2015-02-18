cmake_minimum_required(VERSION 2.8.3)
project(cwru_teleop)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  cwru_msgs
  cwru_base
  joy
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp geometry_msgs joy std_msgs cwru_base cwru_msgs
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
add_executable(teleop_cwru_joy src/teleop_cwru_joy.cpp)

# THIS LINE ADDED TO FIX DEPENDENCY ISSUE FROM CWRU_MSGS
# add_dependencies(teleop_cwru_joy ${catkin_EXPORTED_TARGETS})

target_link_libraries(teleop_cwru_joy ${catkin_LIBRARIES})

