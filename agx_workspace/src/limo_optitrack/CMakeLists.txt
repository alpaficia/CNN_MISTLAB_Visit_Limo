cmake_minimum_required(VERSION 3.5)
project(limo_optitrack)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install entry points
install(PROGRAMS
  limo_optitrack/scripts/limo_pos_publisher.py
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

