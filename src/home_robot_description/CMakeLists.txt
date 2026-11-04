cmake_minimum_required(VERSION 3.8)
project(home_robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/thermal_sensor.py scripts/wait_for_gui.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
