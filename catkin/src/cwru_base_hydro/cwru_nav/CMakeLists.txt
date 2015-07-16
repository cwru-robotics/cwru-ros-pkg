cmake_minimum_required(VERSION 2.8.3)
project(cwru_nav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  amcl
  cwru_base
  filters
  move_base
  nav_msgs
  playsound_recovery
  pluginlib
  roscpp
  rospy
  sensor_msgs
  tf
  tf_conversions
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cwru_nav
#  CATKIN_DEPENDS amcl cwru_base filters move_base nav_msgs playsound_recovery pluginlib roscpp rospy sensor_msgs tf tf_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(cwru_laser_scan_filters src/cwru_laser_scan_filters.cpp)

## Declare a cpp executable
add_executable(map_as_sensor src/map_as_sensor.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(cwru_nav_node cwru_nav_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(map_as_sensor
   ${catkin_LIBRARIES}
)