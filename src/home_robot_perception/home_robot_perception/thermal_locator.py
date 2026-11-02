"""Locate a thermal-image column in the map with the 2D lidar.

The bearing of a pixel column is matched with the lidar ray closest to it.
Both sensors are placed in the map with TF at their own stamps, so turning
does not skew the match. Images wait in a queue until TF has caught up with
their stamp (waiting inside a callback would block TF reception).

Usage: ThermalLocator(node, on_image) subscribes /thermal/image and /scan and
calls on_image(img) for each image once TF covers it; locate(img, col)
returns the (x, y) map point at that column, or None.
"""
import math
from collections import deque

import numpy as np
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import Buffer, TransformListener

HFOV = 0.9599            # thermal horizontal FOV [rad], must match the URDF
WINDOW = 1.0             # bearing window for lidar matching [pixels]
MAX_WAIT = 0.5           # drop queued images older than this vs latest TF [s]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ThermalLocator:
    def __init__(self, node, on_image):
        self.node = node
        self.on_image = on_image
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node)
        node.create_subscription(LaserScan, 'scan', self.scans_append, 10)
        node.create_subscription(Image, 'thermal/image', self.images_append, 10)
        node.create_timer(0.05, self.process)
        self.scans = deque(maxlen=10)
        self.images = deque(maxlen=10)

    def scans_append(self, msg):
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

    def pose(self, frame, stamp):
        """(x, y, yaw) of frame in the map at stamp, or None."""
        try:
            t = self.tf.lookup_transform('map', frame, Time.from_msg(stamp))
        except Exception:  # noqa: BLE001 - TF not available at that stamp
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def locate(self, img, col):
        if not self.scans:
            return None
        t_img = Time.from_msg(img.header.stamp).nanoseconds
        scan = min(self.scans,
                   key=lambda s: abs(Time.from_msg(s.header.stamp).nanoseconds - t_img))
        pix = HFOV / img.width
        bearing = ((img.width - 1) / 2 - col) * pix      # left positive

        cam = self.pose('thermal_link', img.header.stamp)
        lid = self.pose(scan.header.frame_id, scan.header.stamp)
        if None in (cam, lid):
            return None

        # lidar points in map
        r = np.asarray(scan.ranges)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)
        r, a = r[ok], a[ok] + lid[2]
        px, py = lid[0] + r * np.cos(a), lid[1] + r * np.sin(a)
        # bearing of each point as seen from the thermal sensor
        b = np.arctan2(py - cam[1], px - cam[0]) - cam[2]
        b = (b - bearing + math.pi) % (2 * math.pi) - math.pi
        sel = np.abs(b) <= WINDOW * pix
        if not sel.any():
            return None
        i = np.argmin(np.abs(b[sel]))      # ray closest to the bearing
        return float(px[sel][i]), float(py[sel][i])
