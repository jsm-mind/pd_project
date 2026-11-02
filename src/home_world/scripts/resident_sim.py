#!/usr/bin/env python3
"""Simulated resident behavior (not robot perception).

The actor walks to resident_anchor (FollowActor). The heat body
(resident_body: 36.5 C + collision for lidar) marks where the resident stands.

/resident_follow true : body hidden, anchor walks along the robot's track
                        follow_dist behind it (through doors, not walls)
/resident_follow false: anchor walks straight to safe_point (outdoor
                        assembly spot), body is put there
Robot position comes from TF map <- base_footprint.
"""
import os
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')  # gz python msgs

import math  # noqa: E402
from collections import deque  # noqa: E402

import rclpy  # noqa: E402
from gz.msgs10.boolean_pb2 import Boolean  # noqa: E402
from gz.msgs10.pose_pb2 import Pose  # noqa: E402
from gz.transport13 import Node as GzNode  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.time import Time  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

PERIOD = 0.1
SET_POSE = '/world/fire_house_world/set_pose'
HIDDEN_Z = -5.0


class ResidentSim(Node):
    def __init__(self):
        super().__init__('resident_sim')
        self.pos = list(self.declare_parameter('start', [8.1, 6.4]).value)
        self.safe = list(self.declare_parameter('safe_point', [4.3, -2.5]).value)
        self.follow_dist = self.declare_parameter('follow_dist', 1.0).value
        self.speed = self.declare_parameter('speed', 1.0).value
        self.floor_z = self.declare_parameter('floor_z', 0.1).value
        self.gz = GzNode()
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(Bool, 'resident_follow', self.on_follow, 10)
        self.create_timer(PERIOD, self.step)
        self.state = 'idle'     # 'idle' | 'follow' | 'leave'
        self.trail = deque()    # robot positions, oldest first

    def on_follow(self, msg):
        if msg.data and self.state != 'follow':
            self.get_logger().info('Resident follows the robot')
            self.state = 'follow'
            self.trail.clear()
            self.set_pose('resident_body', self.pos, HIDDEN_Z)
        elif not msg.data and self.state == 'follow':
            self.get_logger().info('Resident walks to the assembly point')
            self.state = 'leave'

    def step(self):
        if self.state == 'follow':
            target = self.follow_target()
        elif self.state == 'leave':
            target = self.safe
        else:
            return
        if target is None:
            return
        d = math.dist(self.pos, target)
        step = self.speed * PERIOD
        if d > 1e-3:
            k = min(step, d) / d
            self.pos = [self.pos[0] + (target[0] - self.pos[0]) * k,
                        self.pos[1] + (target[1] - self.pos[1]) * k]
            self.set_pose('resident_anchor', self.pos, self.floor_z)
        if self.state == 'leave' and d <= step:
            self.get_logger().info('Resident at the assembly point')
            self.set_pose('resident_body', self.pos, self.floor_z)
            self.state = 'idle'

    def follow_target(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time(), Duration(seconds=0.05))
        except Exception:  # noqa: BLE001 - TF not ready yet
            return None
        robot = (t.transform.translation.x, t.transform.translation.y)
        if not self.trail or math.dist(self.trail[-1], robot) > 0.1:
            self.trail.append(robot)
        if math.dist(self.pos, robot) <= self.follow_dist:
            return None
        step = self.speed * PERIOD
        while self.trail and math.dist(self.pos, self.trail[0]) < step:
            self.trail.popleft()
        return self.trail[0] if self.trail else robot

    def set_pose(self, name, xy, z):
        req = Pose(name=name)
        req.position.x, req.position.y, req.position.z = xy[0], xy[1], z
        req.orientation.w = 1.0
        self.gz.request(SET_POSE, req, Pose, Boolean, 100)


def main():
    rclpy.init()
    rclpy.spin(ResidentSim())


if __name__ == '__main__':
    main()
