"""Keyboard trigger for mode events (to be replaced by a time-based trigger).

p: start_patrol   c: go_charge   r: reset (EMERGENCY -> CHARGING)
a: ring the fire alarm (simulated alarm device)   q: quit
"""
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

KEYS = {'p': 'start_patrol', 'c': 'go_charge', 'r': 'reset'}


def main():
    rclpy.init()
    node = Node('key_trigger')
    pub = node.create_publisher(String, 'mode_event', 10)
    alarm_pub = node.create_publisher(Empty, 'fire_alarm_trigger', 10)
    print(__doc__)

    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = sys.stdin.read(1)
            if key == 'q':
                break
            if key == 'a':
                alarm_pub.publish(Empty())
                print('sent: fire_alarm_trigger')
            if key in KEYS:
                pub.publish(String(data=KEYS[key]))
                print(f'sent: {KEYS[key]}')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()
