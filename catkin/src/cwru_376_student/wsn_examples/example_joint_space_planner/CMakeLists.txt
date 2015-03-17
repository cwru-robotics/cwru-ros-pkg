cmake_minimum_required(VERSION 2.8.3)
project(example_joint_space_planner)

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
set(CMAKE_BUILD_TYPE Release)

# Libraries
cs_add_library(joint_space_planner src/joint_space_planner.cpp)   

# Executables
cs_add_executable(joint_space_planner_test_main src/joint_space_planner_test_main.cpp)
cs_add_executable(joint_space_planner_test_main2 src/joint_space_planner_test_main2.cpp)
cs_add_executable(test_ik_traj_sender2 src/test_ik_traj_sender2.cpp)
target_link_libraries(joint_space_planner_test_main joint_space_planner)
target_link_libraries(joint_space_planner_test_main2 joint_space_planner)
target_link_libraries(test_ik_traj_sender2 joint_space_planner)
cs_install()
cs_export()
    
