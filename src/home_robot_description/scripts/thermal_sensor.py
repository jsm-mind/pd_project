#!/usr/bin/env python3
"""Emulate a 32x24 MLX90640-class thermal array.

Block-averages the 160x96 Gazebo thermal image (L16, kelvin / resolution)
into 32x24 and publishes temperatures in Celsius as a 32FC1 image.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

GZ_RESOLUTION = 0.1          # must match ThermalSensor <resolution> in the URDF
OUT_W, OUT_H = 32, 24
MIN_C, MAX_C = -40.0, 300.0  # MLX90640 measurement range


class ThermalSensor(Node):
    def __init__(self):
        super().__init__('thermal_sensor')
        self.pub = self.create_publisher(Image, 'thermal/image', 10)
        self.create_subscription(Image, 'thermal/image_full', self.on_image, 10)

    def on_image(self, msg):
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        celsius = raw.astype(np.float32) * GZ_RESOLUTION - 273.15
        bh, bw = msg.height // OUT_H, msg.width // OUT_W
        small = celsius.reshape(OUT_H, bh, OUT_W, bw).mean(axis=(1, 3))
        small = np.clip(small, MIN_C, MAX_C).astype(np.float32)

        out = Image()
        out.header = msg.header
        out.height, out.width = OUT_H, OUT_W
        out.encoding = '32FC1'
        out.step = OUT_W * 4
        out.data = small.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(ThermalSensor())


if __name__ == '__main__':
    main()
