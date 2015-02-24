cmake_minimum_required(VERSION 2.8.3)
project(example_des_state_generator)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# C++0x support - not quite the same as final C++11!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(des_state_generator src/example_des_state_generator.cpp)
cs_add_executable(example_path_sender src/example_path_sender.cpp)
# target_link_library(example my_lib)

cs_install()
cs_export()
    