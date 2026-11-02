"""Resident detection from the 32x24 thermal array + 2D lidar.

Pixels around body temperature (min_c..max_c, not next to fire pixels) are a
person. The bearing of their mean column is matched with the nearest lidar
point in that direction to get the position in the map frame. Both sensors
are placed in the map with TF at their own stamps, so turning does not skew
the match (images wait in a queue until TF has caught up with their stamp).

Input : /thermal/image (32FC1, Celsius), /scan, TF map <- thermal_link / lidar
Output: /person_location (geometry_msgs/PointStamped, map) on every detection
"""
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import Buffer, TransformListener

HFOV = 0.9599            # thermal horizontal FOV [rad], must match the URDF
FIRE_C = 80.0            # pixels next to fire are not people (hot edges)
MIN_PIXELS = 3
WINDOW = 1.0             # bearing window for lidar matching [pixels]
MAX_WAIT = 0.5           # drop queued images older than this vs latest TF [s]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        self.min_c = self.declare_parameter('person_min_c', 30.0).value
        self.max_c = self.declare_parameter('person_max_c', 42.0).value
        self.pub = self.create_publisher(PointStamped, 'person_location', 10)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.create_subscription(Image, 'thermal/image', self.images_append, 10)
        self.create_timer(0.05, self.process)
        self.scans = deque(maxlen=10)
        self.images = deque(maxlen=10)

    def on_scan(self, msg):
        self.scans.append(msg)

    def images_append(self, msg):
        self.images.append(msg)

    def process(self):
        # handle queued images once TF covers their stamp
        while self.images:
            img = self.images[0]
            stamp = Time.from_msg(img.header.stamp)
            if self.tf.can_transform('map', 'thermal_link', stamp):
                self.images.popleft()
                self.on_image(img)
                continue
            try:
                latest = self.tf.lookup_transform('map', 'thermal_link', Time()).header.stamp
            except Exception:  # noqa: BLE001 - TF not ready yet
                latest = None
            if latest is None or (stamp - Time.from_msg(latest)).nanoseconds * 1e-9 > MAX_WAIT:
                self.images.popleft()     # TF missing for too long
                continue
            return

    def lookup(self, source, stamp):
        try:
            t = self.tf.lookup_transform('map', source, Time.from_msg(stamp))
        except Exception:  # noqa: BLE001 - TF not available at that stamp
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def on_image(self, msg):
        if not self.scans:
            return
        t_img = Time.from_msg(msg.header.stamp).nanoseconds
        self.scan = min(self.scans,
                        key=lambda s: abs(Time.from_msg(s.header.stamp).nanoseconds - t_img))
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

        col = np.nonzero(mask)[1].mean()
        pix = HFOV / msg.width
        bearing = ((msg.width - 1) / 2 - col) * pix      # left positive

        cam = self.lookup('thermal_link', msg.header.stamp)
        lid = self.lookup(self.scan.header.frame_id, self.scan.header.stamp)
        if None in (cam, lid):
            return

        # lidar points in map
        r = np.asarray(self.scan.ranges)
        a = self.scan.angle_min + np.arange(len(r)) * self.scan.angle_increment
        ok = np.isfinite(r) & (r > self.scan.range_min) & (r < self.scan.range_max)
        r, a = r[ok], a[ok] + lid[2]
        px, py = lid[0] + r * np.cos(a), lid[1] + r * np.sin(a)
        # bearing of each point as seen from the thermal sensor
        b = np.arctan2(py - cam[1], px - cam[0]) - cam[2]
        b = (b - bearing + math.pi) % (2 * math.pi) - math.pi
        sel = np.abs(b) <= WINDOW * pix
        if not sel.any():
            return
        i = np.argmin(np.abs(b[sel]))      # ray closest to the person's bearing
        out = PointStamped()
        out.header.frame_id = 'map'
        out.header.stamp = msg.header.stamp
        out.point.x, out.point.y = float(px[sel][i]), float(py[sel][i])
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(PersonDetector())
