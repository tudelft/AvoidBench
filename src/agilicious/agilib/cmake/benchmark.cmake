# Download and unpack benchmark at configure time
message(STATUS "Getting Google Benchmark...")

configure_file(cmake/benchmark_download.cmake ${PROJECT_SOURCE_DIR}/externals/benchmark-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/externals/benchmark-download
  OUTPUT_QUIET)
if(result)
  message(FATAL_ERROR "CMake step for benchmark failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/externals/benchmark-download
  OUTPUT_QUIET)
if(result)
  message(FATAL_ERROR "Build step for benchmark failed: ${result}")
endif()

message(STATUS "Google Benchmark downloaded!")

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(benchmark_force_shared_crt ON CACHE INTERNAL "" FORCE)

# Some flags
set(BENCHMARK_ENABLE_INSTALL OFF CACHE INTERNAL "" FORCE)
set(BENCHMARK_DOWNLOAD_DEPENDENCIES OFF CACHE INTERNAL "" FORCE)
set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE INTERNAL "" FORCE)
set(BENCHMARK_ENABLE_TESTING OFF CACHE INTERNAL "" FORCE)
set(GOOGLETEST_PATH ${PROJECT_SOURCE_DIR}/externals/googletest-src)

# Add benchmark directly to our build.
add_subdirectory(${PROJECT_SOURCE_DIR}/externals/benchmark-src
                 ${PROJECT_SOURCE_DIR}/externals/benchmark-build
                 EXCLUDE_FROM_ALL)

include_directories("${benchmark_SOURCE_DIR}/include")
