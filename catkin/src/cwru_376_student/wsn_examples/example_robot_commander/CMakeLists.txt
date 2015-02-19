cmake_minimum_required(VERSION 2.8.3)
project(example_robot_commander)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(robot_commander src/example_robot_commander.cpp)
cs_add_executable(example_cwruBot_commander src/example_cwruBot_commander.cpp)
cs_add_executable(halt_commander src/halt_commander.cpp)
cs_add_executable(vel_scheduler src/vel_scheduler.cpp)
cs_add_executable(interactive_robot_commander src/interactive_robot_commander.cpp)
cs_add_executable(interactive_robot_commander_v2 src/interactive_robot_commander_v2.cpp)
# target_link_library(example my_lib)
 
cs_install()
cs_export()
    