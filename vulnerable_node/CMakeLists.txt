cmake_minimum_required(VERSION 3.8)
project(vulnerable_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(vulnerable_target vulnerable_target.cpp)
ament_target_dependencies(vulnerable_target rclcpp std_msgs)

install(TARGETS
  vulnerable_target
  DESTINATION lib/${PROJECT_NAME})

ament_package()
