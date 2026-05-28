cmake_minimum_required(VERSION 3.8)
project(team_b_decision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)

# L14 scaffold: install only BT XML; C++ targets come in L16.
install(
  DIRECTORY bt_xml
  DESTINATION share/${PROJECT_NAME}
)

# Placeholder for L16 — uncomment when bt_runner.cpp is implemented.
# add_executable(bt_runner src/bt_runner.cpp)
# ament_target_dependencies(bt_runner rclcpp behaviortree_cpp_v3 nav2_behavior_tree nav2_msgs)
# install(TARGETS bt_runner DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
