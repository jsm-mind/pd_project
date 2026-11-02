#!/bin/bash
# Usage: ./run.sh [build] [fire_spot]
#   fire_spot: random (default), stove_1, stove_2, tv_stand,
#              nightstand_bedroom1, nightstand_bedroom2, utility_room
cd "$(dirname "$(readlink -f "$0")")"
if [ "$1" = build ]; then
    shift
    colcon build || exit 1
fi
source install/setup.bash
ros2 launch home_robot_nav bringup.launch.py fire_spot:="${1:-random}"
