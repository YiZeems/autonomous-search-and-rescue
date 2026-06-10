cmake_minimum_required(VERSION 3.8)
project(rescue_decision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(bt_runner src/bt_runner.cpp)
ament_target_dependencies(bt_runner
  rclcpp behaviortree_cpp_v3 std_msgs nav_msgs geometry_msgs)

install(TARGETS bt_runner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY bt_xml launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
