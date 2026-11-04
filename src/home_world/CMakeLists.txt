cmake_minimum_required(VERSION 3.8)
project(home_world)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

# Actor system: standing animation while FollowActor keeps the actor still
add_library(actor_stand SHARED src/actor_stand.cc)
target_link_libraries(actor_stand gz-sim8::gz-sim8 gz-plugin2::register)
install(TARGETS actor_stand LIBRARY DESTINATION lib)

install(DIRECTORY worlds models launch
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/resident_sim.py scripts/fire_alarm_sim.py scripts/fire_sim.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
