"""Fire detection from the 32x24 thermal array.

Input : /thermal/image (32FC1, Celsius), /scan, TF, /mode
        /fire_check 'start' | 'stop' - also detect once in EMERGENCY (scenario 2)
Output: /mode_event 'fire_detected' (once, in PATROL or CHARGING; not for /fire_check)
        /fire_location (geometry_msgs/PoseStamped, transient_local) -
        fire position in the map: hottest pixel's bearing + farthest lidar range
        within +-1 pixel (ThermalLocator); robot position if the lidar match fails
        /fire_seen (std_msgs/Float32) - peak Celsius, the first time a fire is in
        view during each EMERGENCY (any step; for the relay screen alert)
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32, String

from home_robot_perception.thermal_locator import ThermalLocator

ACTIVE_MODES = ('PATROL', 'CHARGING')


class FireDetector(Node):
    def __init__(self):
        super().__init__('fire_detector')
        self.threshold = self.declare_parameter('fire_threshold_c', 80.0).value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.event_pub = self.create_publisher(String, 'mode_event', 10)
        self.location_pub = self.create_publisher(PoseStamped, 'fire_location', latched)
        self.seen_pub = self.create_publisher(Float32, 'fire_seen', 10)
        self.create_subscription(String, 'mode', self.on_mode, latched)
        self.create_subscription(String, 'fire_check', self.on_check, 10)
        self.locator = ThermalLocator(self, self.on_image)

        self.mode = None
        self.sent = False   # one 'fire_detected' per mode entry
        self.checking = False
        self.seen = False   # one 'fire_seen' per mode entry

    def on_mode(self, msg):
        if msg.data != self.mode:
            self.sent = False
            self.seen = False
        self.mode = msg.data

    def on_check(self, msg):
        self.checking = msg.data == 'start'

    def on_image(self, msg):
        if self.mode == 'EMERGENCY' and not self.seen:
            peak = float(np.frombuffer(msg.data, dtype=np.float32).max())
            if peak >= self.threshold:
                self.seen = True
                self.seen_pub.publish(Float32(data=peak))
        if not self.checking and (self.mode not in ACTIVE_MODES or self.sent):
            return
        temps = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        row, col = np.unravel_index(np.argmax(temps), temps.shape)
        peak = float(temps[row, col])
        if peak < self.threshold:
            return

        check, self.checking = self.checking, False
        self.sent = True
        self.get_logger().info(
            f'FIRE detected: {peak:.1f} C at pixel (col {col}, row {row}) in {self.mode}')
        xy = self.locator.locate(msg, col, farthest=True)
        if xy is None:
            robot = self.locator.pose('base_footprint', msg.header.stamp)
            xy = robot[:2] if robot else None
            self.get_logger().warn('Lidar match failed; using robot position as fire location')
        if xy is not None:
            loc = PoseStamped()
            loc.header.frame_id = 'map'
            loc.header.stamp = msg.header.stamp
            loc.pose.position.x, loc.pose.position.y = xy
            loc.pose.orientation.w = 1.0
            self.location_pub.publish(loc)
            self.get_logger().info(f'Fire location: ({xy[0]:.2f}, {xy[1]:.2f})')
        if not check:
            self.event_pub.publish(String(data='fire_detected'))


def main():
    rclpy.init()
    rclpy.spin(FireDetector())
