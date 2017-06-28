cmake_minimum_required(VERSION 2.8.3)
project(arc_lane_following_autopilot)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

## Add libraries.
cs_add_library(ipm_lib ../arc_lane_tracking_tools/src/inverse_perspective_mapping/inverse_perspective_mapping.cpp)
cs_add_library(linedetector_lib ../arc_lane_tracking_tools/src/line_detection/line_detection.cpp)
cs_add_library(ransac_lib ../arc_lane_tracking_tools/src/ransac_fitting/ransac_fitting.cpp)

## Add executables.
cs_add_executable(lane_tracker_exec src/lanetracking.cpp)

## Link libraries to executables.
target_link_libraries(lane_tracker_exec ipm_lib linedetector_lib)

cs_install()

cs_export()
