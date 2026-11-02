"""Stands in for the house smoke detector the robot hears (no sound in the simulation).

Publishes /alarm/active (latched, on change only). The alarm carries no position.
  - Fire spawned at a candidate with a detector -> alarm on after a random delay
    (smoke reaching the detector). Candidates without a detector never raise it.
  - Fire extinguished -> alarm off after off_delay.
  - false_alarm: alarm on false_alarm_start s after /patrol/started, for
    false_alarm_duration s (<= 0: never ends), with no fire.
Uses the fire ground truth (/fire_sim/spawned): simulation-side node only.
"""
import random

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty

MATCH_DISTANCE = 0.1   # [m] a spawned fire this close to a candidate is that candidate


class AlarmSimulator(Node):

    def __init__(self):
        super().__init__('alarm_simulator')
        p = self.declare_parameter
        self.on_delay_min = p('on_delay_min', 20.0).value
        self.on_delay_max = p('on_delay_max', 40.0).value
        self.off_delay = p('off_delay', 10.0).value
        self.false_alarm = p('false_alarm', False).value
        self.false_alarm_start = p('false_alarm_start', 60.0).value
        self.false_alarm_duration = p('false_alarm_duration', 90.0).value
        flat = p('candidates', Parameter.Type.DOUBLE_ARRAY).value or []
        self.candidates = list(zip(flat[0::2], flat[1::2]))
        self.has_detector = list(p('has_detector', Parameter.Type.BOOL_ARRAY).value or [])
        if len(self.has_detector) != len(self.candidates):
            self.get_logger().warn(f'has_detector has {len(self.has_detector)} entries for '
                                   f'{len(self.candidates)} candidates, missing ones have a detector')

        self.fire_alarm = False     # alarm raised by a real fire
        self.false_active = False   # alarm raised by the false alarm scenario
        self.active = None
        self.fire_timer = None      # pending fire alarm on/off
        self.false_timer = None
        self.started = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.alarm_pub = self.create_publisher(Bool, '/alarm/active', latched)
        self.create_subscription(PointStamped, '/fire_sim/spawned', self.on_spawned, latched)
        self.create_subscription(Empty, '/fire_sim/extinguished', self.on_extinguished, 10)
        self.create_subscription(Empty, '/patrol/started', self.on_patrol_started, latched)
        self.publish()

    # ------------------------------------------------------------ fire alarm
    def detector_at(self, x, y):
        for i, (cx, cy) in enumerate(self.candidates):
            if abs(cx - x) <= MATCH_DISTANCE and abs(cy - y) <= MATCH_DISTANCE:
                return self.has_detector[i] if i < len(self.has_detector) else True
        self.get_logger().warn(f'Fire at ({x:.2f}, {y:.2f}) is not a candidate, '
                               'assuming a detector')
        return True

    def on_spawned(self, msg):
        x, y = msg.point.x, msg.point.y
        self.cancel_fire_timer()
        if not self.detector_at(x, y):
            self.get_logger().info(f'Fire at ({x:.2f}, {y:.2f}): no detector there, no alarm')
            return
        delay = random.uniform(self.on_delay_min, self.on_delay_max)
        self.get_logger().info(f'Fire at ({x:.2f}, {y:.2f}): alarm in {delay:.1f} s')
        self.fire_timer = self.create_timer(delay, lambda: self.set_fire_alarm(True))

    def on_extinguished(self, _msg):
        self.cancel_fire_timer()
        if self.fire_alarm:
            self.get_logger().info(f'Fire extinguished: alarm off in {self.off_delay:.1f} s')
            self.fire_timer = self.create_timer(self.off_delay, lambda: self.set_fire_alarm(False))

    def set_fire_alarm(self, on):
        self.cancel_fire_timer()
        self.fire_alarm = on
        self.publish()

    def cancel_fire_timer(self):
        if self.fire_timer is not None:
            self.destroy_timer(self.fire_timer)
            self.fire_timer = None

    # ----------------------------------------------------------- false alarm
    def on_patrol_started(self, _msg):
        if self.started or not self.false_alarm:
            return
        self.started = True
        self.get_logger().info(f'False alarm scenario: alarm in {self.false_alarm_start:.1f} s')
        self.false_timer = self.create_timer(self.false_alarm_start, self.false_alarm_on)

    def false_alarm_on(self):
        self.destroy_timer(self.false_timer)
        self.false_timer = None
        self.false_active = True
        self.publish()
        if self.false_alarm_duration > 0.0:
            self.false_timer = self.create_timer(self.false_alarm_duration, self.false_alarm_off)

    def false_alarm_off(self):
        self.destroy_timer(self.false_timer)
        self.false_timer = None
        self.false_active = False
        self.publish()

    # ---------------------------------------------------------------- output
    def publish(self):
        active = self.fire_alarm or self.false_active
        if active == self.active:
            return
        self.active = active
        self.alarm_pub.publish(Bool(data=active))
        self.get_logger().info('Alarm ON' if active else 'Alarm OFF')


def main():
    rclpy.init()
    try:
        rclpy.spin(AlarmSimulator())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
