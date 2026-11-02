"""Fire detection from the 32x24 thermal array.

Input : /thermal/image (32FC1, Celsius), /mode, /amcl_pose
Output: /mode_event 'fire_detected' (once, in PATROL or CHARGING)
        /fire_location (geometry_msgs/PoseStamped, transient_local) -
        robot AMCL pose at the moment of detection
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String

ACTIVE_MODES = ('PATROL', 'CHARGING')


class FireDetector(Node):
    def __init__(self):
        super().__init__('fire_detector')
        self.threshold = self.declare_parameter('fire_threshold_c', 80.0).value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.event_pub = self.create_publisher(String, 'mode_event', 10)
        self.location_pub = self.create_publisher(PoseStamped, 'fire_location', latched)
        self.create_subscription(String, 'mode', self.on_mode, latched)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl, 10)
        self.create_subscription(Image, 'thermal/image', self.on_image, 10)

        self.mode = None
        self.amcl = None
        self.sent = False   # one 'fire_detected' per mode entry

    def on_mode(self, msg):
        if msg.data != self.mode:
            self.sent = False
        self.mode = msg.data

    def on_amcl(self, msg):
        self.amcl = msg

    def on_image(self, msg):
        if self.mode not in ACTIVE_MODES or self.sent:
            return
        temps = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        row, col = np.unravel_index(np.argmax(temps), temps.shape)
        peak = float(temps[row, col])
        if peak < self.threshold:
            return

        self.sent = True
        self.get_logger().info(
            f'FIRE detected: {peak:.1f} C at pixel (col {col}, row {row}) in {self.mode}')
        if self.amcl is not None:
            loc = PoseStamped(header=self.amcl.header, pose=self.amcl.pose.pose)
            self.location_pub.publish(loc)
            p = loc.pose.position
            self.get_logger().info(f'Fire location (robot pose): ({p.x:.2f}, {p.y:.2f})')
        else:
            self.get_logger().warn('No AMCL pose yet; fire location not published')
        self.event_pub.publish(String(data='fire_detected'))


def main():
    rclpy.init()
    rclpy.spin(FireDetector())
