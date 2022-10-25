project(rpg_common)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(OpenCV REQUIRED)

add_definitions(-std=c++11)

FILE(GLOB_RECURSE HEADERS "include/*")

list(APPEND SOURCES
  src/cv_type.cc
  src/cv_utils.cpp
  src/fs.cpp
  src/gnuplot.cpp
  src/hamming_distance.cpp
  src/sim3.cpp
  src/thread_timer.cpp
  src/time_enforcer.cpp
  src/triangulate.cpp)

cs_add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

catkin_add_gtest(test_triangulate test/test_triangulate.cpp)
target_link_libraries(test_triangulate ${PROJECT_NAME})

catkin_add_gtest(test_cv_utils test/test_cv_utils.cpp)
target_link_libraries(test_cv_utils ${PROJECT_NAME})

catkin_add_gtest(test_save_load_eigen test/test_save_load_eigen.cpp)
target_link_libraries(test_save_load_eigen ${PROJECT_NAME})

cs_install()
cs_export()
