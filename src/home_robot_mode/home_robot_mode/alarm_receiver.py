"""Fire alarm reception (alarm device -> robot link).

Input : /fire_alarm     (PoseStamped, map) - position of the ringing alarm
        /mode
Output: /alarm_location (PoseStamped, transient_local) - target for scenario 2 step 5
        /mode_event 'fire_alarm' (in PATROL or CHARGING)
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

ACTIVE_MODES = ('PATROL', 'CHARGING')


class AlarmReceiver(Node):
    def __init__(self):
        super().__init__('alarm_receiver')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.event_pub = self.create_publisher(String, 'mode_event', 10)
        self.location_pub = self.create_publisher(PoseStamped, 'alarm_location', latched)
        self.create_subscription(String, 'mode', self.on_mode, latched)
        self.create_subscription(PoseStamped, 'fire_alarm', self.on_alarm, 10)
        self.mode = None

    def on_mode(self, msg):
        self.mode = msg.data

    def on_alarm(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        if self.mode not in ACTIVE_MODES:
            self.get_logger().info(f'Alarm at ({x:.2f}, {y:.2f}) ignored in {self.mode}')
            return
        self.get_logger().info(f'FIRE ALARM received at ({x:.2f}, {y:.2f}) in {self.mode}')
        self.location_pub.publish(msg)
        self.event_pub.publish(String(data='fire_alarm'))


def main():
    rclpy.init()
    rclpy.spin(AlarmReceiver())
