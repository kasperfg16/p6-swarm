cmake_minimum_required(VERSION 3.5)
project(process_robot_navigation)

find_package(rclpy REQUIRED)
find_package(ament_cmake REQUIRED)
ament_python_install_package(scripts)

install(
  DIRECTORY launch config maps scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/robot_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
