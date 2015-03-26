cmake_minimum_required(VERSION 2.8.3)
project(example_irb120_IM_interface)

find_package(catkin_simple REQUIRED)
find_package(Eigen REQUIRED)
find_package(irb120_ik REQUIRED)
#find_package(example_joint_space_planner REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   

# Executables
# cs_add_executable(example src/example.cpp)
cs_add_executable(example_irb120_IM_interface src/example_irb120_IM_interface.cpp)

# target_link_library(example my_lib)

cs_install()
cs_export()
    