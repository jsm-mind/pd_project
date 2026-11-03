#!/usr/bin/env python3
"""Simulated fire outbreak (not robot perception).

The fire model is spawned at the chosen fire spot a random delay
(delay_min..delay_max s) after the robot first starts patrolling, or right away
when the fire alarm is triggered before that (an alarm means a fire).

Input : /mode (std_msgs/String), /fire_alarm_trigger (std_msgs/Empty)
Output: 'fire' model in Gazebo (removed later by the suppression node)
        /fire_started (std_msgs/Empty) - to the alarm simulator (smoke starts spreading)
"""
import os
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')  # gz python msgs

import random  # noqa: E402

import rclpy  # noqa: E402
from gz.msgs10.boolean_pb2 import Boolean  # noqa: E402
from gz.msgs10.entity_factory_pb2 import EntityFactory  # noqa: E402
from gz.transport13 import Node as GzNode  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import Empty, String  # noqa: E402

CREATE = '/world/fire_house_world/create'


class FireSim(Node):
    def __init__(self):
        super().__init__('fire_sim')
        self.spot = self.declare_parameter('fire_spot', 'tv_stand').value
        self.xyz = self.declare_parameter('position', [4.95, 2.60, 0.60]).value
        self.delay_min = self.declare_parameter('delay_min', 10.0).value
        self.delay_max = self.declare_parameter('delay_max', 40.0).value
        self.gz = GzNode()
        self.started_pub = self.create_publisher(Empty, 'fire_started', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'mode', self.on_mode, latched)
        self.create_subscription(Empty, 'fire_alarm_trigger', lambda _: self.spawn(), 10)
        self.timer = None
        self.spawned = False

    def on_mode(self, msg):
        if msg.data != 'PATROL' or self.timer is not None or self.spawned:
            return
        delay = random.uniform(self.delay_min, self.delay_max)
        self.get_logger().info(f'Fire will break out at {self.spot} in {delay:.0f} s')
        self.timer = self.create_timer(delay, self.spawn)

    def spawn(self):
        if self.timer is not None:
            self.timer.cancel()
        if self.spawned:
            return
        self.spawned = True
        req = EntityFactory(sdf_filename='model://fire', name='fire')
        req.pose.position.x, req.pose.position.y, req.pose.position.z = self.xyz
        self.gz.request(CREATE, req, EntityFactory, Boolean, 2000)
        self.started_pub.publish(Empty())
        self.get_logger().info(f'FIRE broke out at {self.spot} {tuple(self.xyz)}')


def main():
    rclpy.init()
    rclpy.spin(FireSim())


if __name__ == '__main__':
    main()
