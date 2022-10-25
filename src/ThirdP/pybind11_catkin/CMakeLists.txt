cmake_minimum_required(VERSION 3.4)
project(pybind11_catkin)

find_package(catkin REQUIRED)
include(ExternalProject)

ExternalProject_Add(pybind11_src
  URL "https://github.com/pybind/pybind11/archive/v2.6.1.zip"
  UPDATE_COMMAND ""
  CMAKE_ARGS -DPYBIND11_NOPYTHON=TRUE
             -DPYBIND11_TEST=OFF
             -DPYBIND11_INSTALL=ON
             -DCMAKE_INSTALL_PREFIX=${PROJECT_BINARY_DIR}/install
             -DCMAKE_INSTALL_INCLUDEDIR=${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}
)
# We need to copy the entire content of the cmake/pybind11 folder to the correct place but with a different structure:
ExternalProject_Add_Step(pybind11_src CopyToDevel
  COMMENT "Copying to devel"
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_BINARY_DIR}/install/share/cmake/pybind11/ ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake
  DEPENDEES install
)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME})
catkin_package(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}
  CFG_EXTRAS pybind11_catkin.cmake
)

install(
  DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# We glob the entire _original_ pybind11 cmake directory since there would be a few
# files in the devel-share directory which we do *not* want to copy as they will be
# regenerated in the install space:
#  - pybind11_catkin.cmake
#  - pybind11_catkinConfig-version.cmake
#  - pybind11_catkinConfig.cmake
# file(GLOB PYBIND11_CMAKE_FILES ${PROJECT_BINARY_DIR}/install/share/cmake/pybind11/*.cmake)

set(PYBIND11_CMAKE_FILES
  ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/FindPythonLibsNew.cmake
  ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/pybind11Config.cmake
  ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/pybind11ConfigVersion.cmake
  ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/pybind11NewTools.cmake
  ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/pybind11Targets.cmake
  ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/pybind11Common.cmake
  ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/pybind11Tools.cmake
)

install(
  FILES ${PYBIND11_CMAKE_FILES}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
)
