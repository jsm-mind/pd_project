"""Evacuation guidance outputs: voice and floor arrow projector.

Input : /voice     (std_msgs/String) - spoken line (terminal + spd-say)
        /projector (std_msgs/Bool)   - floor arrow in front of the robot on/off
Output: Gazebo marker (visual only) drawn from TF map <- base_footprint
"""
import os
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')  # gz python msgs

import math  # noqa: E402
import subprocess  # noqa: E402

import rclpy  # noqa: E402
from gz.msgs10.boolean_pb2 import Boolean  # noqa: E402
from gz.msgs10.marker_pb2 import Marker  # noqa: E402
from gz.msgs10.marker_v_pb2 import Marker_V  # noqa: E402
from gz.transport13 import Node as GzNode  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.time import Time  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

MARKER_SRV = '/marker_array'   # GUI scene; '/marker' is one-way (no reply)
FLOOR_Z = 0.105
AHEAD = 0.45        # arrow tail distance in front of the robot center [m]
# arrow outline in its own frame (x forward), as triangles
ARROW = [
    (0.0, -0.06), (0.30, -0.06), (0.30, 0.06),
    (0.0, -0.06), (0.30, 0.06), (0.0, 0.06),
    (0.30, -0.15), (0.50, 0.0), (0.30, 0.15),
]


class Guidance(Node):
    def __init__(self):
        super().__init__('guidance')
        self.gz = GzNode()
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(String, 'voice', self.on_voice, 10)
        self.create_subscription(Bool, 'projector', self.on_projector, 10)
        self.timer = None

    def on_voice(self, msg):
        bar = '-' * 60
        print(f'{bar}\n[음성 안내] {msg.data}\n{bar}', flush=True)
        try:
            subprocess.Popen(['spd-say', '-l', 'ko', msg.data],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError:
            pass

    def on_projector(self, msg):
        if msg.data and self.timer is None:
            self.get_logger().info('Projector on')
            self.timer = self.create_timer(0.2, self.draw)
        elif not msg.data and self.timer is not None:
            self.get_logger().info('Projector off')
            self.timer.cancel()
            self.timer = None
            self.send(Marker(ns='projector', id=1, action=Marker.DELETE_MARKER))

    def draw(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time(), Duration(seconds=0.1))
        except Exception:  # noqa: BLE001 - TF not ready yet
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        m = Marker(ns='projector', id=1, action=Marker.ADD_MODIFY, type=Marker.TRIANGLE_LIST)
        m.pose.position.x = t.transform.translation.x + AHEAD * math.cos(yaw)
        m.pose.position.y = t.transform.translation.y + AHEAD * math.sin(yaw)
        m.pose.position.z = FLOOR_Z
        m.pose.orientation.z = math.sin(yaw / 2)
        m.pose.orientation.w = math.cos(yaw / 2)
        for c in (m.material.ambient, m.material.diffuse, m.material.emissive):
            c.r, c.g, c.b, c.a = 0.1, 1.0, 0.2, 1.0
        for x, y in ARROW:
            p = m.point.add()
            p.x, p.y, p.z = x, y, 0.0
        self.send(m)

    def send(self, marker):
        req = Marker_V()
        req.marker.append(marker)
        self.gz.request(MARKER_SRV, req, Marker_V, Boolean, 300)


def main():
    rclpy.init()
    rclpy.spin(Guidance())
