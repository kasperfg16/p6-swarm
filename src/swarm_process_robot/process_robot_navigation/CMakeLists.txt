cmake_minimum_required(VERSION 3.5)
project(process_robot_navigation)

find_package(rclpy REQUIRED)
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()