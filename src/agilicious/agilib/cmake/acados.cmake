# Download and unpack acados at configure time
message(STATUS "Getting Acados...")

configure_file(
  cmake/acados_download.cmake
  ${PROJECT_SOURCE_DIR}/externals/acados-download/CMakeLists.txt)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/externals/acados-download
  OUTPUT_QUIET
  ERROR_QUIET)
if(result)
  message(FATAL_ERROR "Download of Acados failed: ${result}")
  message(STATUS "${PROJECT_SOURCE_DIR}/externals/acados-download")
endif()


execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/externals/acados-download/)
if(result)
  message(FATAL_ERROR "Build step for Acados failed: ${result}")
endif()

message(STATUS "Acados downloaded and built!")

add_library(hpipm_lib SHARED IMPORTED)
set_target_properties(hpipm_lib PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES ${PROJECT_SOURCE_DIR}/externals/acados-src/external/hpipm/include
  IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/externals/acados-src/lib/libhpipm.so)

add_library(blasfeo_lib SHARED IMPORTED)
set_target_properties(blasfeo_lib PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES ${PROJECT_SOURCE_DIR}/externals/acados-src/external/blasfeo/include
  IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/externals/acados-src/lib/libblasfeo.so)

add_library(acados_lib SHARED IMPORTED)
set_target_properties(acados_lib PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES ${PROJECT_SOURCE_DIR}/externals/acados-src/include
  IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/externals/acados-src/lib/libacados.so)

set(acados_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/externals/acados-src/include"
  "${PROJECT_SOURCE_DIR}/externals/acados-src/external/blasfeo/include"
  "${PROJECT_SOURCE_DIR}/externals/acados-src/external/hpipm/include")

set(acados_LIB_DIR "${PROJECT_SOURCE_DIR}/externals/acados-src/lib")


add_library(acados_agi INTERFACE IMPORTED GLOBAL)
target_link_libraries(acados_agi INTERFACE hpipm_lib blasfeo_lib acados_lib)


