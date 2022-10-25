cmake_minimum_required(VERSION 2.8.3)
project(rpg_single_board_io)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} src/gpio.cpp src/adc.cpp)
cs_add_executable(gpio_interrupt_example src/gpio_interrupt_example.cpp src/gpio.cpp)
cs_add_executable(voltage_reader src/voltage_reader.cpp src/adc.cpp)

cs_install()
cs_export()
