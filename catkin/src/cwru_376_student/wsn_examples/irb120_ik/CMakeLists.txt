cmake_minimum_required(VERSION 2.8.3)
project(irb120_ik)


find_package(catkin_simple REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_simple()
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})


# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   
cs_add_library(irb120_kinematics src/irb120_kinematics.cpp) 

# Executables
# cs_add_executable(example src/example.cpp)
# target_link_library(example my_lib)
cs_add_executable(irb120_kinematics_test_main src/irb120_kinematics_test_main.cpp)
target_link_libraries(irb120_kinematics_test_main irb120_kinematics)

cs_add_executable(reachability_from_above src/reachability_from_above.cpp)
target_link_libraries(reachability_from_above irb120_kinematics)

cs_install()
cs_export()
    