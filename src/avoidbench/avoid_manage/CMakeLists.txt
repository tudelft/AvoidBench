project(avoid_manage)

cmake_minimum_required(VERSION 3.0.0)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
find_package(avoidmetrics REQUIRED)
        
catkin_simple(ALL_DEPS_REQUIRED)
include(cmake/yaml.cmake)
# Setup Default Build Type as Release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()

# Setup Architcture-specific Flags
if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
    message(STATUS "Using ARMv7 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
elseif ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    message(STATUS "Using ARM aarch64 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
else ()
    set(CMAKE_CXX_ARCH_FLAGS " -march=native")
endif ()

# Setup General C++ Flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_STACK_ALLOCATION_LIMIT=1048576")
# otherwise double free or corruption (out) error when running racing or motion_planning example
add_compile_options(-O3)

# Setup Release and Debug flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_ARCH_FLAGS} -Wall -DNDEBUG -fPIC -fopenmp")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -g -fopenmp")
#pilot

include_directories(${avoidmetrics_INCLUDE_DIR}
                    yaml-cpp)
cs_add_library(avoid_manage
    src/avoid_manage.cpp
)

target_link_libraries(avoid_manage
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${avoidmetrics_LIBRARIES}
  stdc++fs
)

cs_add_executable(avoid_manage_node
   src/avoid_manage_node.cpp
)

target_link_libraries(avoid_manage_node
  avoid_manage
  ${OpenCV_LIBRARIES}
  stdc++fs
)

cs_add_library(rl_test
    src/rl_test.cpp
)

cs_add_executable(rl_test_node
   src/avoid_manage_node.cpp
)

target_link_libraries(rl_test_node
  rl_test
  ${OpenCV_LIBRARIES}
  stdc++fs
)

catkin_install_python(PROGRAMS scripts/plot_metrics.py
				scripts/avoid_manage.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Finish
cs_install()
cs_export()
