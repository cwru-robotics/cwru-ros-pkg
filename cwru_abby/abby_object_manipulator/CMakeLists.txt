cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_add_boost_directories()

rosbuild_add_library(${PROJECT_NAME}_tools src/tools/mechanism_interface.cpp
                                           src/tools/grasp_marker_publisher.cpp
                                           src/tools/vector_tools.cpp
                                           src/tools/convert_functions.cpp
                                           include/object_manipulator/tools/msg_helpers.h
                                           include/object_manipulator/tools/configuration_loader.h
                                           include/object_manipulator/tools/arm_configurations.h
                                           include/object_manipulator/tools/hand_description.h
                                           include/object_manipulator/tools/camera_configurations.h
                                           src/tools/shape_tools.cpp
										   src/tools/ik_tester_fast.cpp
                                           )
rosbuild_link_boost(${PROJECT_NAME}_tools thread)

rosbuild_add_library(${PROJECT_NAME}_grasp_execution src/tools/mechanism_interface.cpp
						     src/grasp_execution/grasp_executor.cpp
                                                     src/grasp_execution/simple_grasp_executor.cpp
                                                     src/grasp_execution/grasp_executor_with_approach.cpp
                                                     src/grasp_execution/grasp_tester_fast.cpp
                                                     src/grasp_execution/reactive_grasp_executor.cpp
                                                     src/grasp_execution/unsafe_grasp_executor.cpp 
						     src/grasp_execution/approach_lift_grasp.cpp)
						   
rosbuild_add_library(${PROJECT_NAME}_place_execution src/place_execution/place_executor.cpp
                                                     src/place_execution/descend_retreat_place.cpp
                                                     src/place_execution/place_tester_fast.cpp)
target_link_libraries(${PROJECT_NAME}_grasp_execution ${PROJECT_NAME}_tools)
target_link_libraries(${PROJECT_NAME}_place_execution ${PROJECT_NAME}_tools)

rosbuild_add_library(${PROJECT_NAME} src/object_manipulator.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_tools
                                      ${PROJECT_NAME}_grasp_execution
                                      ${PROJECT_NAME}_place_execution)

rosbuild_add_executable(object_manipulator_node nodes/object_manipulator_node.cpp)
target_link_libraries(object_manipulator_node ${PROJECT_NAME}_tools
                                              ${PROJECT_NAME}_grasp_execution
                                              ${PROJECT_NAME}_place_execution
                                              ${PROJECT_NAME})

