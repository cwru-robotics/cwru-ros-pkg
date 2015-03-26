cmake_minimum_required(VERSION 2.8.3)
project(example_interactive_marker)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(IM_example src/IM_example.cpp)
cs_add_executable(IM_6dof_example src/IM_6dof_example.cpp)
cs_add_executable(IM_example2 src/IM_example2.cpp)
cs_add_executable(IM_example3 src/IM_example3.cpp)
cs_add_executable(path_display src/path_display.cpp)
cs_add_executable(marker_example src/marker_example.cpp)
cs_add_executable(marker_listener src/marker_listener.cpp)
cs_add_executable(simple_marker_listener src/simple_marker_listener.cpp)
cs_add_executable(marker_listener_v2 src/marker_listener_v2.cpp)
cs_add_executable(example_marker_user_app src/example_marker_user_app.cpp)
cs_add_executable(example_marker_user_app_v2 src/example_marker_user_app_v2.cpp)
# target_link_library(example my_lib)

cs_install()
cs_export()
    
