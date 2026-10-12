cmake_minimum_required(VERSION 3.5)

project(pcl_visualizer)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
find_package(Qt6 QUIET COMPONENTS Concurrent OpenGL Widgets)
if (NOT Qt6_FOUND)
  find_package(Qt5 COMPONENTS Concurrent OpenGL Widgets REQUIRED)
  set(QTX Qt5)
else()
  set(QTX Qt6)
endif()

find_package(VTK REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(NLOPT REQUIRED nlopt)
find_package(PCL 1.7.1 REQUIRED)

# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

set(project_SOURCES main.cpp pclviewer.cpp
        logic/get_cones_cloud.h
        logic/cones_recog.cpp
        logic/cones_recog.h
        logic/distinguish_left_right.cpp
        logic/distinguish_left_right.h
        logic/get_cones_cloud.cpp
        logic/get_cones_cloud.h
        logic/get_obstacle_cloud.cpp
        logic/get_obstacle_cloud.h
        logic/racingline_nlopt.cpp
        logic/racingline_nlopt.h
)

add_executable(${PROJECT_NAME} ${project_SOURCES})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${QTX}::Widgets)
target_include_directories(${PROJECT_NAME} PRIVATE ${NLOPT_INCLUDE_DIRS} )
target_link_directories(${PROJECT_NAME} PRIVATE ${NLOPT_LIBRARY_DIRS})
add_definitions(${NLOPT_CFLAGS_OTHER})
target_link_libraries(${PROJECT_NAME} ${NLOPT_LIBRARIES})
# Evaluation of the left/right cone classification (synthetic tracks + optional real cloud).
add_executable(eval_left_right
        tests/eval_left_right.cpp
        tests/synthetic_tracks.h
        logic/distinguish_left_right.cpp
        logic/get_cones_cloud.cpp
        logic/cones_recog.cpp
)
target_include_directories(eval_left_right PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS})
target_link_libraries(eval_left_right ${PCL_LIBRARIES})

enable_testing()
add_test(NAME left_right_synthetic COMMAND eval_left_right --seeds 50 --report ${CMAKE_CURRENT_BINARY_DIR}/left_right_report.html)

# Ground-truth tests of the racing line (known optimum, interpolation, bounds).
add_executable(test_racing_line
        tests/test_racing_line.cpp
        tests/synthetic_tracks.h
        logic/racingline_nlopt.cpp
)
target_include_directories(test_racing_line PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS} ${NLOPT_INCLUDE_DIRS})
target_link_directories(test_racing_line PRIVATE ${NLOPT_LIBRARY_DIRS})
target_link_libraries(test_racing_line ${PCL_LIBRARIES} ${NLOPT_LIBRARIES})
add_test(NAME racing_line COMMAND test_racing_line)

# End-to-end ground-truth evaluation on simulated scans (detection, sides, racing line).
add_executable(eval_pipeline
        tests/eval_pipeline.cpp
        tests/lidar_sim.h
        tests/report_html.h
        logic/get_cones_cloud.cpp
        logic/cones_recog.cpp
        logic/distinguish_left_right.cpp
        logic/racingline_nlopt.cpp
)
target_include_directories(eval_pipeline PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS} ${NLOPT_INCLUDE_DIRS})
target_link_directories(eval_pipeline PRIVATE ${NLOPT_LIBRARY_DIRS})
target_link_libraries(eval_pipeline ${PCL_LIBRARIES} ${NLOPT_LIBRARIES})
# Not a ctest: it reports metrics and a visual report, without pass/fail thresholds.

# Verification of the cone-recognition ICP (known rotations, simulated views).
add_executable(test_cone_icp
        tests/test_cone_icp.cpp
        tests/lidar_sim.h
        logic/cones_recog.cpp
)
target_include_directories(test_cone_icp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS})
target_link_libraries(test_cone_icp ${PCL_LIBRARIES})
add_test(NAME cone_icp COMMAND test_cone_icp ${CMAKE_CURRENT_SOURCE_DIR}/external_files/ideal_cone.ply)
