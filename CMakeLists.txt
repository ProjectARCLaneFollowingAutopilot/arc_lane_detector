cmake_minimum_required(VERSION 2.8.3)
project(arc_lane_following_autopilot)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)


cs_add_executable(lane_detector_exec src/main.cpp)

cs_install()

cs_export()
