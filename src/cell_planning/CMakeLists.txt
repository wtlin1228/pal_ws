cmake_minimum_required(VERSION 3.8)
project(cell_planning)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(cell_planning INTERFACE)                      # header-only
target_include_directories(cell_planning INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(cell_planning INTERFACE Eigen3::Eigen ${geometry_msgs_TARGETS} tf2_eigen::tf2_eigen)

add_executable(show_pattern src/show_pattern.cpp)
ament_target_dependencies(show_pattern rclcpp visualization_msgs)
target_link_libraries(show_pattern cell_planning)
install(TARGETS show_pattern DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS cell_planning EXPORT cell_planning_targets)
ament_export_targets(cell_planning_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 geometry_msgs tf2_eigen)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry cell_planning)
  ament_add_gtest(test_grasps test/test_grasps.cpp)
  target_link_libraries(test_grasps cell_planning)
  ament_add_gtest(test_patterns test/test_patterns.cpp)
  target_link_libraries(test_patterns cell_planning)
  ament_add_gtest(test_mixed test/test_mixed.cpp)
  target_link_libraries(test_mixed cell_planning)
endif()
ament_package()
