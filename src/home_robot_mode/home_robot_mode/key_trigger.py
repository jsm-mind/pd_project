"""Keyboard trigger for mode events (to be replaced by a time-based trigger).

p: start_patrol   c: go_charge   r: reset (EMERGENCY -> CHARGING)   q: quit
"""
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

KEYS = {'p': 'start_patrol', 'c': 'go_charge', 'r': 'reset'}


def main():
    rclpy.init()
    node = Node('key_trigger')
    pub = node.create_publisher(String, 'mode_event', 10)
    print(__doc__)

    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = sys.stdin.read(1)
            if key == 'q':
                break
            if key in KEYS:
                pub.publish(String(data=KEYS[key]))
                print(f'sent: {KEYS[key]}')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()
