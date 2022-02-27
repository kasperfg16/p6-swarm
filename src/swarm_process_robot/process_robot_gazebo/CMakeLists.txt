cmake_minimum_required(VERSION 3.5)
project(process_robot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
ament_python_install_package(scripts)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/robot_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
