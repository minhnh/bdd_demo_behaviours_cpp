cmake_minimum_required(VERSION 3.20)
project(bdd_collab_bhv_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(bdd_ros2_interfaces REQUIRED)
find_package(coord2b REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${coord2b_INCLUDE_DIRS}
)

add_executable(collab_pickplace_node
  src/collab_pickplace_node.cpp
  src/fsm_behaviours.cpp
  src/conversions.cpp
)
target_include_directories(collab_pickplace_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(collab_pickplace_node PUBLIC
  c_std_99 cxx_std_17 cxx_std_20
)
target_link_libraries(
  collab_pickplace_node
  coord2b
  ${bdd_ros2_interfaces_TARGETS}
  ${rclcpp_TARGETS}
  ${rclcpp_action_TARGETS}
  ${rclcpp_components_TARGETS}
  yaml-cpp
)

install(TARGETS collab_pickplace_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
