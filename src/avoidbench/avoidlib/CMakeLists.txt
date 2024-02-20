cmake_minimum_required(VERSION 3.0)

project(avoidlib VERSION 0.1.0)

message(STATUS "======================  !Flightmare!  ======================")

################################################################################
# Options
################################################################################
option(BUILD_TESTS "Building the tests" ON)
option(BUILD_UNITY_BRIDGE_TESTS "Building the Unity Bridge tests" ON)
option(BUILD_BENCH "Building the benchmark." OFF)
option(ENABLE_FAST "Build with optimizations for speed" ON)
option(ENABLE_BLAS "Build using BLAS and LAPACK libraries" OFF)
option(ENABLE_PARALLEL "Build using openmp parallelization" ON)
option(EIGEN_FROM_SYSTTEM "Use the system-provided Eigen" ON)
set(
  EIGEN_ALTERNATIVE "" CACHE STRING
  "Path to alternative Eigen, autodownload if blank"
)

# Including dependencies 
include(cmake/pybind11.cmake)
include(cmake/yaml.cmake)

message(STATUS "======> Setup Dependencies")
if(EIGEN_FROM_SYSTTEM)
  find_package(Eigen3 3.3.4 QUIET)
  if(EIGEN3_FOUND)
    message(STATUS "Using system provided Eigen.")
    message(${EIGEN3_INCLUDE_DIR})
  else()
    message(STATUS "No sufficient Eigen version (3.3.4) found.")
    message(STATUS "Restoring to download Eigen sources.")
    include(cmake/eigen.cmake)
  endif()
elseif(EIGEN_ALTERNATIVE STREQUAL "")
  include(cmake/eigen.cmake)
else()
  set(EIGEN_INCLUDE_DIR ${EIGEN_ALTERNATIVE})
endif()
message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")

find_package(CUDA REQUIRED)
set(CUDA_NVCC_FLAGS
        ${CUDA_NVCC_FLAGS};
        -O3
        -gencode=arch=compute_52,code=sm_52
        -gencode=arch=compute_61,code=sm_61
        -gencode=arch=compute_62,code=sm_62
        -gencode=arch=compute_75,code=sm_75
        -gencode=arch=compute_86,code=sm_86
        )

message("CUDA Libs: ${CUDA_LIBRARIES}")
message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
message("CUDA curand library: ${CUDA_curand_LIBRARY}")

# Including dependencies 
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Open3D)

if(ENABLE_BLAS)
  set(BLA_VENDOR "Generic")
  find_package(BLAS REQUIRED)
  if(BLAS_FOUND)
    message(STATUS "Found BLAS: ${BLAS_LIBRARIES}")
  else()
    message(ERROR "Could not enable BLAS because BLAS was not found")
  endif()
  find_package(LAPACK REQUIRED)
  if(LAPACK_FOUND)
    message(STATUS "Found Lapack: ${LAPACK_LIBRARIES}")
  else()
    message(ERROR "Could not enable LAPACK because LAPACK was not found")
  endif()
endif()

################################################################################
# Setup Compilation
################################################################################
message(STATUS "======> Setup Compilation")

# Set default build type
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

# Add c++ flags 
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -fPIC -Wall -DNDEBUG  -fopenmp")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -fPIC -Wall -g -fopenmp")
set(CMAKE_CXX_STANDARD 17)

# Architectural flags
if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
  message(STATUS "Using ARMv7 optimized flags!")
  set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
elseif("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
  message(STATUS "Using ARM aarch64 optimized flags!")
  set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
else()
  set(CMAKE_CXX_ARCH_FLAGS " -march=native")
endif()

# Optimized flags
if(ENABLE_FAST)
  message(STATUS "Enabling fast optimization flags!")
  set(CMAKE_CXX_FAST_FLAGS " -Ofast")
else()
  set(CMAKE_CXX_FAST_FLAGS " -O0")
endif()

# BLAS Flags
if(BLAS_FOUND AND LAPACK_FOUND)
  message(STATUS "Enabling BLAS and LAPACK")
  set(CMAKE_CXX_BLAS_FLAGS " -DEIGEN_USE_BLAS -DEIGEN_USE_LAPACK -DEIGEN_USE_LAPACKE")
else()
  set(CMAKE_CXX_BLAS_FLAGS "")
endif()

# Summarize Flags
set(CMAKE_CXX_FLAGS_RELEASE
  "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FAST_FLAGS} ${CMAKE_CXX_ARCH_FLAGS} ${CMAKE_CXX_PAR_FLAGS}")
string(REPLACE "-DNDEBUG" "" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
message(STATUS "The activated CXX RELEASE configuration is:\n ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "The activated CXX DEBUG configuration is:\n ${CMAKE_CXX_FLAGS_DEBUG}")

################################################################################
# Specify Build Resources
################################################################################
message(STATUS "======>  Setup Build  ")

# Create file lists for avoidlib source
file(GLOB_RECURSE AVOIDLIB_SOURCES
  src/bridges/unity_bridge.cpp
  src/dynamics/*.cpp
  src/objects/*.cpp
  src/sensors/*.cpp
  src/envs/*.cpp
  src/vision_envs/*.cpp
  src/common/*.cpp
  src/controller/*.cpp
  src/pointclouds/*.cpp
)

file(GLOB_RECURSE AVOIDBRIDGE_SOURCES
  src/avoidbench_bridge.cpp
)

file(GLOB_RECURSE FLIGHTLIB_GYM_SOURCES
  src/wrapper/pybind_wrapper.cpp 
)

# Library and Executables
include_directories(include ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${Open3D_INCLUDE_DIRS})

if(NOT AVOIDLIB_SOURCES)
	set(LIBRARY_NAME)
else()
  # avoidlib
  add_library(${PROJECT_NAME} ${AVOIDLIB_SOURCES})
  target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    ${Open3D_LIBRARIES}
    yaml-cpp
    zmq
    zmqpp
    stdc++fs)
  set(LIBRARY_NAME ${PROJECT_NAME})
endif()

  
cuda_add_library(sgm_gpu
      src/sgm_gpu/sgm_gpu.cu
      src/sgm_gpu/costs.cu
      src/sgm_gpu/hamming_cost.cu
      src/sgm_gpu/left_right_consistency.cu
      src/sgm_gpu/median_filter.cu)

if(DEFINED CATKIN_DEVEL_PREFIX)
  find_package(catkin_simple REQUIRED)     
  catkin_package(
          LIBRARIES
          CATKIN_DEPENDS
  )
  catkin_simple(ALL_DEPS_REQUIRED)


  cs_add_library(avoidbench_bridge ${AVOIDBRIDGE_SOURCES})
  target_link_libraries(avoidbench_bridge
                        sgm_gpu 
                        ${PROJECT_NAME})

  # avoidlib_bridge (python3 binding with Pybind11)
  pybind11_add_module(avoidbridge MODULE
    src/wrapper/avoidbench_wrapper.cpp
  )

  if(EIGEN3_FOUND)
    target_include_directories(avoidbridge PRIVATE
      # ${PROJECT_SOURCE_DIR}/externals/pybind11-src/include
      ${PYBIND11_INCLUDE_DIR}
      ${PROJECT_SOURCE_DIR}/include
      ${EIGEN3_INCLUDE_DIR})
  else()
    target_include_directories(avoidbridge PRIVATE
      # ${PROJECT_SOURCE_DIR}/externals/pybind11-src/include
      ${PYBIND11_INCLUDE_DIR}
      ${PROJECT_SOURCE_DIR}/externals/eigen/eigen3 
      ${PROJECT_SOURCE_DIR}/include)
  endif()
  target_link_libraries(avoidbridge PRIVATE
                        avoidbench_bridge
                        ${LIBRARY_NAME})

  set_target_properties(avoidbridge
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/../)

  cs_install()
  cs_export()

else()

  if(FLIGHTLIB_GYM_SOURCES)
    # flightlib_gym (python3 binding with Pybind11)
    pybind11_add_module(flightgym MODULE 
      ${FLIGHTLIB_GYM_SOURCES})
    if(EIGEN3_FOUND)
      target_include_directories(flightgym PRIVATE
        # ${PROJECT_SOURCE_DIR}/externals/pybind11-src/include
        ${PYBIND11_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR})  
    else()
      target_include_directories(flightgym PRIVATE
        # ${PROJECT_SOURCE_DIR}/externals/pybind11-src/include
        ${PYBIND11_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/externals/eigen/eigen3 
        ${PROJECT_SOURCE_DIR}/include)  
    endif()
    message("${LIBRARY_NAME}")
    target_link_libraries(flightgym PRIVATE sgm_gpu ${LIBRARY_NAME})
  endif()
  
endif()
