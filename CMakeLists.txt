cmake_minimum_required(VERSION 3.16)

project(planbench VERSION 1.0.0 LANGUAGES CXX)

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(PLANBENCH_BUILD_TESTS "Build unit tests" ON)

add_library(planbench
  src/occupancy_grid.cpp
  src/astar.cpp
  src/rrt_star.cpp
  src/state_lattice.cpp
  src/map_generator.cpp
)
target_include_directories(planbench PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(planbench PUBLIC cxx_std_17)

add_executable(planbench_benchmark src/benchmark.cpp)
target_link_libraries(planbench_benchmark PRIVATE planbench)

function(planbench_warnings target)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Werror)
endfunction()

planbench_warnings(planbench)
planbench_warnings(planbench_benchmark)

if(PLANBENCH_BUILD_TESTS)
  enable_testing()
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  add_executable(planbench_tests
    tests/test_astar.cpp
    tests/test_collision.cpp
    tests/test_rrt_star.cpp
    tests/test_state_lattice.cpp
    tests/test_map_generator.cpp
  )
  target_link_libraries(planbench_tests PRIVATE planbench GTest::gtest_main)
  planbench_warnings(planbench_tests)

  include(GoogleTest)
  gtest_discover_tests(planbench_tests PROPERTIES DISCOVERY_TIMEOUT 60)
endif()
