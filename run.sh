#!/bin/bash
# Usage: ./run.sh [build] [call] [rviz] [noalarm] [fire_spot]
#   build    : colcon build first
#   call     : really dial the 119 stand-in number (default: log only)
#   rviz     : also open RViz (map, robot pose, path)
#   noalarm  : the fire alarm is broken (no sound, no signal) -> the robot must find the fire (scenario 1)
#   fire_spot (fire breaks out 10-40 s after patrol start): random (default), stove_1, stove_2, tv_stand,
#              nightstand_bedroom1, nightstand_bedroom2, utility_room
cd "$(dirname "$(readlink -f "$0")")"
call=false
rviz=false
alarm=true
spot=random
for arg in "$@"; do
    case "$arg" in
        build) colcon build || exit 1 ;;
        call) call=true ;;
        rviz) rviz=true ;;
        noalarm) alarm=false ;;
        *) spot="$arg" ;;
    esac
done
source install/setup.bash
ros2 launch home_robot_nav bringup.launch.py fire_spot:="$spot" call:="$call" rviz:="$rviz" alarm:="$alarm"
