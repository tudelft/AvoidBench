cmake_minimum_required(VERSION 3.0.0)

project(benchmark-download NONE)

include(ExternalProject)
ExternalProject_Add(benchmark
  GIT_REPOSITORY    https://github.com/google/benchmark.git
  GIT_TAG           master
  SOURCE_DIR        "${PROJECT_SOURCE_DIR}/externals/benchmark-src"
  BINARY_DIR        "${PROJECT_SOURCE_DIR}/externals/benchmark-build"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ""
  INSTALL_COMMAND   ""
  TEST_COMMAND      ""
  UPDATE_DISCONNECTED ON
)