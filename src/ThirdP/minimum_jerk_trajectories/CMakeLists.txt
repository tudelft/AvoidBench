cmake_minimum_required(VERSION 2.8.3)
project(minimum_jerk_trajectories)

find_package(catkin_simple REQUIRED)
catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

cs_add_library(${PROJECT_NAME} src/RapidTrajectoryGenerator.cpp src/SingleAxisTrajectory.cpp)

cs_export()
cs_install()
