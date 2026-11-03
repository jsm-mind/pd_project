#!/usr/bin/env python3
"""Simulated fire alarm devices (government-issued, linked to the robot).

One alarm per room, on the ceiling above open floor (map frame), so the robot
can stand under it. The alarm in the room of the spawned fire spot rings
alarm_delay s after the fire breaks out (time for the smoke to reach it; the
smoke itself is not simulated), or right away when triggered by hand.

Input : /fire_started       (std_msgs/Empty) - from fire_sim
        /fire_alarm_trigger (std_msgs/Empty) - from the key window ('a')
Output: /fire_alarm (geometry_msgs/PoseStamped, map) - ringing alarm's position
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Empty

# room -> alarm position (x, y) in map
ALARMS = {
    'living_room': (3.20, 2.55),
    'kitchen': (2.52, 6.28),
    'utility_room': (4.80, 6.60),
    'bedroom1': (8.00, 1.45),
    'bedroom2': (8.14, 5.25),
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
        self.alarm_delay = self.declare_parameter('alarm_delay', 10.0).value
        self.pub = self.create_publisher(PoseStamped, 'fire_alarm', 10)
        self.create_subscription(Empty, 'fire_alarm_trigger', self.on_trigger, 10)
        self.create_subscription(Empty, 'fire_started', self.on_fire, 10)
        self.timer = None
        self.rang = False

    def on_fire(self, _):
        if self.timer is None:
            self.get_logger().info(f'Smoke spreading; alarm in {self.room} rings in {self.alarm_delay:.0f} s')
            self.timer = self.create_timer(self.alarm_delay, self.on_delay)

    def on_delay(self):
        self.timer.cancel()
        if not self.rang:     # already rung by hand ('a' before the fire)
            self.on_trigger(None)

    def on_trigger(self, _):
        self.rang = True
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
