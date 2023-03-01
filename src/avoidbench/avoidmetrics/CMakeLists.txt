project(avoidmetrics)

cmake_minimum_required(VERSION 3.0.0)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 3.3.4 QUIET)
find_package(PCL REQUIRED)

if(EIGEN3_FOUND)
  message(STATUS "Using system provided Eigen.")
  message(${EIGEN3_INCLUDE_DIR})
else()
  message(STATUS "No sufficient Eigen version (3.3.4) found.")
  message(STATUS "Restoring to download Eigen sources.")
  include(cmake/eigen.cmake)
endif()

find_package(Open3D)
if (Open3D_FOUND)
    message("Found Open3D ${Open3D_VERSION}")
    list(APPEND Open3D_LIBRARIES dl)
    # link_directories must be before add_executable
    link_directories(${Open3D_LIBRARY_DIRS})
else ()
    message("Open3D not found")
endif ()

include(cmake/yaml.cmake)
include(cmake/pybind11.cmake)
message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")

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

cs_add_library(Metrics
    src/Metrics.cpp
    src/Mission.cpp
    src/environment.cpp
    src/astar_path_finding.cpp
)

if(EIGEN3_FOUND)
  target_include_directories(Metrics PUBLIC
    # ${PROJECT_SOURCE_DIR}/externals/pybind11-src/include
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${Open3D_INCLUDE_DIRS}
    yaml-cpp)  
else()
  target_include_directories(Metrics PUBLIC
    # ${PROJECT_SOURCE_DIR}/externals/pybind11-src/include
    ${PROJECT_SOURCE_DIR}/externals/eigen/eigen3
    ${PROJECT_SOURCE_DIR}/include
    ${Open3D_INCLUDE_DIRS}
    yaml-cpp)  
endif()

target_link_libraries(Metrics
  stdc++fs
  ${Open3D_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

cs_add_executable(read_pcl
    src/read_pcl.cpp
)
target_include_directories(read_pcl PUBLIC
  # ${PROJECT_SOURCE_DIR}/externals/pybind11-src/include
  ${PROJECT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS})
  
target_link_libraries(read_pcl
  ${PCL_LIBRARIES}
  stdc++fs
)

pybind11_add_module(avoidmetrics MODULE 
    src/wrapper/metrics_wrapper.cpp)
    
if(EIGEN3_FOUND)
  target_include_directories(avoidmetrics PUBLIC
    # ${PROJECT_SOURCE_DIR}/externals/pybind11-src/include
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${Open3D_INCLUDE_DIRS}
    )  
else()
  target_include_directories(avoidmetrics PUBLIC
    # ${PROJECT_SOURCE_DIR}/externals/pybind11-src/include
    ${PROJECT_SOURCE_DIR}/externals/eigen/eigen3 #pybind11 use #include <Eigen/Core>, however, flightmare use #include <eigen3/Eigen/Core>
    ${PROJECT_SOURCE_DIR}/include
    ${Open3D_INCLUDE_DIRS}
    )  
endif()

target_link_libraries(avoidmetrics PRIVATE 
                      Metrics 
                      yaml-cpp)

set_target_properties(avoidmetrics
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/../)

# Finish
cs_install()
cs_export()
