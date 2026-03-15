cmake_minimum_required(VERSION 3.20)
project(bdd_demo_behaviours_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(bdd_ros2_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_executable(human_pickplace_mockup_node
  src/human_pickplace_mockup_node.cpp
  src/conversions.cpp
)
target_include_directories(human_pickplace_mockup_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(human_pickplace_mockup_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(
  human_pickplace_mockup_node
  ${bdd_ros2_interfaces_TARGETS}
  ${rclcpp_TARGETS}
  ${rclcpp_action_TARGETS}
  ${rclcpp_components_TARGETS}
)

install(TARGETS human_pickplace_mockup_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
