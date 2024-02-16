cmake_minimum_required(VERSION 3.0.0)

project(acados-project)

include(ExternalProject)
ExternalProject_Add(ACADOS_PROJECT
  GIT_REPOSITORY    https://github.com/uzh-rpg/acados.git
  GIT_TAG           master
  SOURCE_DIR        "${PROJECT_SOURCE_DIR}/externals/acados-src"
  INSTALL_DIR       "${PROJECT_SOURCE_DIR}/externals/acados-src"
  CMAKE_ARGS        "-DCMAKE_C_FLAGS=${CMAKE_C_FLAGS}"
                    "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}"
                    "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
                    "-DCMAKE_INSTALL_PREFIX=${PROJECT_SOURCE_DIR}/externals/acados-src"
)
