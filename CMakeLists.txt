cmake_minimum_required(VERSION 3.8)
project(basic_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(basic_robot_interfaces REQUIRED)

add_executable(lidar src/lidarpublisher.cpp)
ament_target_dependencies(lidar rclcpp basic_robot_interfaces)

install(TARGETS
  lidar
  DESTINATION lib/${PROJECT_NAME})

ament_package()
