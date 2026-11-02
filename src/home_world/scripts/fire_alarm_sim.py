#!/usr/bin/env python3
"""Simulated fire alarm devices (government-issued, linked to the robot).

One alarm per room, at the room center (map frame). The alarm in the room of
the spawned fire spot rings when triggered (smoke reaching it is not simulated).

Input : /fire_alarm_trigger (std_msgs/Empty) - from the key window ('a')
Output: /fire_alarm (geometry_msgs/PoseStamped, map) - ringing alarm's position
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Empty

# room -> alarm position (x, y) in map
ALARMS = {
    'living_room': (2.60, 2.19),
    'kitchen': (2.51, 6.26),
    'utility_room': (4.80, 7.12),
    'bedroom1': (8.00, 1.45),
    'bedroom2': (9.07, 6.32),
}
# fire spot (world.launch.py FIRE_SPOTS) -> room
SPOT_ROOM = {
    'stove_1': 'kitchen',
    'stove_2': 'kitchen',
    'tv_stand': 'living_room',
    'nightstand_bedroom1': 'bedroom1',
    'nightstand_bedroom2': 'bedroom2',
    'utility_room': 'utility_room',
}


class FireAlarmSim(Node):
    def __init__(self):
        super().__init__('fire_alarm_sim')
        self.room = SPOT_ROOM[self.declare_parameter('fire_spot', 'tv_stand').value]
        self.pub = self.create_publisher(PoseStamped, 'fire_alarm', 10)
        self.create_subscription(Empty, 'fire_alarm_trigger', self.on_trigger, 10)

    def on_trigger(self, _):
        x, y = ALARMS[self.room]
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)
        self.get_logger().info(f'ALARM ringing in {self.room} ({x:.2f}, {y:.2f})')


def main():
    rclpy.init()
    rclpy.spin(FireAlarmSim())


if __name__ == '__main__':
    main()
