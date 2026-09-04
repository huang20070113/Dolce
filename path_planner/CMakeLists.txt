cmake_minimum_required(VERSION 3.16)
project(path_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(path_planner_core src/planner.cpp)
target_include_directories(path_planner_core PUBLIC include)

add_executable(path_planner src/main.cpp)
target_link_libraries(path_planner PRIVATE path_planner_core)

include(CTest)
if(BUILD_TESTING)
  add_executable(path_planner_tests tests/planner_tests.cpp)
  target_link_libraries(path_planner_tests PRIVATE path_planner_core)
  add_test(NAME path_planner_tests COMMAND path_planner_tests)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(path_planner_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(path_planner PRIVATE -Wall -Wextra -Wpedantic)
  if(BUILD_TESTING)
    target_compile_options(path_planner_tests PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()
