# Setup catkin simple
find_package(catkin_simple REQUIRED)
catkin_simple()

add_definitions(-std=c++17)

# Library and Executables
if (ACADOS_SOURCES)
    cs_add_library(acados_mpc ${ACADOS_SOURCES})
    target_compile_options(acados_mpc PRIVATE -Wno-error -Wno-register)
    target_link_libraries(acados_mpc acados_agi)
    target_include_directories(acados_mpc PUBLIC
            include/agilib/controller/mpc/acados/
            )
endif ()


cs_add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${BLAS_LIBRARIES}
  ${LAPACK_LIBRARIES}
  ${LAPACKE_LIBRARIES}
  acados_mpc
)

target_compile_options(${PROJECT_NAME} PRIVATE
  -fno-finite-math-only
  -Wall
  -Werror
  -Wpedantic
  -Wunused
  -Wno-unused-parameter
  -Wundef
  -Wcast-align
  -Wmissing-include-dirs
  # -Wnon-virtual-dtor
  -Wredundant-decls
  -Wodr
  -Wunreachable-code
  -Wno-unknown-pragmas
)

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE
    -Wmissing-declarations
    # To keep the compiler calm about Eigen
    -Wno-sign-conversion
    -Wno-implicit-int-float-conversion
    -Wno-c99-extensions
    -Wno-implicit-int-conversion
  )
endif()

# Build tests
if(BUILD_TESTS)
  catkin_add_gtest(agilib_tests ${TEST_SOURCES})
  target_link_libraries(agilib_tests ${PROJECT_NAME} gtest gtest_main)
endif()

# Build benchmarks
if(BUILD_BENCH AND benchmark_catkin_FOUND)
  cs_add_executable(agilib_benchmarks ${BENCH_SOURCES})
  target_link_libraries(agilib_benchmarks ${PROJECT_NAME})
endif()

# Finish catkin simple
cs_install()
cs_export()
