"""Resident detection from the 32x24 thermal array + 2D lidar.

Pixels around body temperature (min_c..max_c, not next to fire pixels) are a
person; their mean column is located in the map with ThermalLocator.

Input : /thermal/image (32FC1, Celsius), /scan, TF
Output: /person_location (geometry_msgs/PointStamped, map) on every detection
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from home_robot_perception.thermal_locator import ThermalLocator

FIRE_C = 80.0            # pixels next to fire are not people (hot edges)
MIN_PIXELS = 3


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        self.min_c = self.declare_parameter('person_min_c', 30.0).value
        self.max_c = self.declare_parameter('person_max_c', 42.0).value
        self.pub = self.create_publisher(PointStamped, 'person_location', 10)
        self.locator = ThermalLocator(self, self.on_image)

    def on_image(self, msg):
        temps = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        mask = (temps >= self.min_c) & (temps <= self.max_c)
        fire = temps >= FIRE_C
        if fire.any():
            near = np.zeros_like(fire)
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    near |= np.roll(np.roll(fire, dr, 0), dc, 1)
            mask &= ~near
        if mask.sum() < MIN_PIXELS:
            return

        xy = self.locator.locate(msg, np.nonzero(mask)[1].mean())
        if xy is None:
            return
        out = PointStamped()
        out.header.frame_id = 'map'
        out.header.stamp = msg.header.stamp
        out.point.x, out.point.y = xy
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(PersonDetector())
