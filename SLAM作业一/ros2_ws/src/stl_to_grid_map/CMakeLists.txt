cmake_minimum_required(VERSION 3.16)
project(stl_to_grid_map VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(stl_grid_core
  src/stl_mesh.cpp
  src/grid_converter.cpp
)
target_include_directories(stl_grid_core PUBLIC include)

if(MSVC)
  target_compile_options(stl_grid_core PRIVATE /W4 /permissive-)
else()
  target_compile_options(stl_grid_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(stl_to_grid_cli src/stl_to_grid_cli.cpp)
target_link_libraries(stl_to_grid_cli PRIVATE stl_grid_core)
add_executable(verify_map src/verify_map.cpp)

include(CTest)
if(BUILD_TESTING)
  add_executable(stl_grid_core_tests test/stl_grid_core_tests.cpp)
  target_link_libraries(stl_grid_core_tests PRIVATE stl_grid_core)
  add_test(NAME stl_grid_core_tests COMMAND stl_grid_core_tests)
endif()

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(nav_msgs REQUIRED)

  add_executable(stl_to_grid_node src/stl_to_grid_node.cpp)
  target_link_libraries(stl_to_grid_node stl_grid_core)
  ament_target_dependencies(stl_to_grid_node rclcpp nav_msgs)

  install(TARGETS
    stl_grid_core
    stl_to_grid_cli
    stl_to_grid_node
    verify_map
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY include/ DESTINATION include)
  install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
  ament_export_include_directories(include)
  ament_export_libraries(stl_grid_core)
  ament_package()
endif()
