#!/bin/bash
# Usage: ./run.sh [build] [call] [fire_spot]
#   build    : colcon build first
#   call     : really dial the 119 stand-in number (default: log only)
#   fire_spot: random (default), stove_1, stove_2, tv_stand,
#              nightstand_bedroom1, nightstand_bedroom2, utility_room
cd "$(dirname "$(readlink -f "$0")")"
call=false
spot=random
for arg in "$@"; do
    case "$arg" in
        build) colcon build || exit 1 ;;
        call) call=true ;;
        *) spot="$arg" ;;
    esac
done
source install/setup.bash
ros2 launch home_robot_nav bringup.launch.py fire_spot:="$spot" call:="$call"
