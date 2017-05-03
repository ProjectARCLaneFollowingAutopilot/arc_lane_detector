cmake_minimum_required(VERSION 2.8.3)
project(arc_lane_following_autopilot)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(ipm_lib src/inverse_perspective_mapping.cpp)

cs_add_executable(arc_lane_detection_exec src/main.cpp)

target_link_libraries(arc_lane_detection_exec ipm_lib)

cs_install()

cs_export()
