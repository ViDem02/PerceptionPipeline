cmake_minimum_required(VERSION 4.0)
project(second_prj_setup)

set(CMAKE_CXX_STANDARD 20)

find_package(PCL 1.3 REQUIRED)
add_library(odometry STATIC odometry.cpp odometry.h)
target_include_directories(odometry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS})
target_link_libraries(odometry PUBLIC ${PCL_LIBRARIES})

add_executable(sample_consensus main.cpp)
target_link_libraries(sample_consensus odometry)

# add_executable(second_prj_setup matrix_transform.cpp)

# Ground-truth test: known synthetic motion applied to first.pcd.
add_executable(test_odometry tests/test_odometry.cpp)
target_link_libraries(test_odometry odometry)

enable_testing()
add_test(NAME odometry_ground_truth COMMAND test_odometry "${CMAKE_CURRENT_SOURCE_DIR}/external files/first.pcd")
