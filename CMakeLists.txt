cmake_minimum_required(VERSION 2.8.3)
project(arc_lane_following_autopilot)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(ipm_lib ../arc_lane_tracking_tools/src/inverse_perspective_mapping/inverse_perspective_mapping.cpp)
cs_add_library(linedetector_lib ../arc_lane_tracking_tools/src/line_detection/line_detection.cpp)
cs_add_library(ransac_lib ../arc_lane_tracking_tools/src/ransac_fitting/ransac_fitting.cpp)

cs_add_executable(test_nikhilesh_exec test_src/test_nikhilesh.cpp)
cs_add_executable(test_ransac_exec test_src/test_ransac.cpp)
##The final executable.
cs_add_executable(lane_tracker_exec src/lanetracking.cpp)
cs_add_executable(test_ipm_exec test_src/test_ipm.cpp)


target_link_libraries(test_nikhilesh_exec ipm_lib)
##target_link_libraries(test_nikhilesh_exec linedetector_lib)
target_link_libraries(test_ransac_exec ransac_lib)
target_link_libraries(lane_tracker_exec ipm_lib linedetector_lib)
target_link_libraries(test_ipm_exec ipm_lib)

cs_install()

cs_export()
