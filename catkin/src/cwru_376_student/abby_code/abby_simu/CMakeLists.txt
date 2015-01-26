cmake_minimum_required(VERSION 2.8.3)
project(abby_simu)

find_package(catkin_simple REQUIRED)

catkin_simple()
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables
#cs_add_executable(abby_simu src/abby_simu.cpp)
cs_add_executable(abby_simu_v1 src/abby_simu_v1.cpp)
# target_link_library(example my_lib)

cs_install()
cs_export()
    