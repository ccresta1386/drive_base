cmake_minimum_required(VERSION 3.5)
project(drive_base)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_executable(drive_base src/mecanum_drive.cpp include/roboclaw_ros.cpp include/roboclaw.c)

ament_target_dependencies(drive_base
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2)


install(TARGETS drive_base
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
  
ament_export_include_directories(include)
ament_export_dependencies(std_msgs
  nav_msgs
  goemetry_msgs
  tf2)
ament_package()

