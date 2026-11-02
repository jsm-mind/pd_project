cmake_minimum_required(VERSION 3.8)
project(home_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models launch
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/resident_sim.py scripts/fire_alarm_sim.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
