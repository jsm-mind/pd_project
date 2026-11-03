"""Robot mode state machine: PATROL / CHARGING / EMERGENCY.

Input : /mode_event (std_msgs/String) - event names below
Output: /mode       (std_msgs/String, transient_local) - current mode
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

PATROL = 'PATROL'
CHARGING = 'CHARGING'
EMERGENCY = 'EMERGENCY'

# (current mode, event) -> next mode
TRANSITIONS = {
    (CHARGING, 'start_patrol'): PATROL,
    (PATROL, 'docked'): CHARGING,   # sent by patrol node after return to charger
    (PATROL, 'fire_detected'): EMERGENCY,
    (CHARGING, 'fire_detected'): EMERGENCY,
    (PATROL, 'fire_alarm'): EMERGENCY,
    (CHARGING, 'fire_alarm'): EMERGENCY,
    (EMERGENCY, 'emergency_done'): CHARGING,  # sent by scenario node after suppression
    (EMERGENCY, 'reset'): CHARGING,  # demo/test only
}


class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, 'mode', latched)
        self.create_subscription(String, 'mode_event', self.on_event, 10)
        self.mode = CHARGING
        self.publish()

    def on_event(self, msg):
        nxt = TRANSITIONS.get((self.mode, msg.data))
        if nxt is None:
            # events not used for transitions (e.g. go_charge) are handled elsewhere
            if any(ev == msg.data for _, ev in TRANSITIONS):
                self.get_logger().warn(f'Ignored event "{msg.data}" in {self.mode}')
            return
        self.get_logger().info(f'{self.mode} --{msg.data}--> {nxt}')
        self.mode = nxt
        self.publish()

    def publish(self):
        self.pub.publish(String(data=self.mode))


def main():
    rclpy.init()
    rclpy.spin(ModeManager())
