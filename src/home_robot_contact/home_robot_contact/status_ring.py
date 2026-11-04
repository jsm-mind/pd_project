"""Status ring on the top cover: emissive color and blinking follow the current mode/step.

Input : /robot_status        (std_msgs/String, latched) - JSON from home_robot_mode robot_status
Output: material of the ring visual via Gazebo UserCommands visual_config, sent only
        when the shown color changes (blink = one request per toggle, wall clock).
"""
import os
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')  # gz python msgs

import json  # noqa: E402
import time  # noqa: E402

import rclpy  # noqa: E402
from gz.msgs10.boolean_pb2 import Boolean  # noqa: E402
from gz.msgs10.empty_pb2 import Empty  # noqa: E402
from gz.msgs10.scene_pb2 import Scene  # noqa: E402
from gz.msgs10.visual_pb2 import Visual  # noqa: E402
from gz.transport13 import Node as GzNode  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402

WORLD = '/world/fire_house_world'
MODEL = 'robot'
RING = 'status_ring_visual'   # URDF visual name; lumped name only contains it

BLUE = (0.0, 0.35, 1.0)
WHITE = (1.0, 1.0, 1.0)
RED = (1.0, 0.0, 0.0)
GREEN = (0.1, 1.0, 0.2)       # same as the floor arrow
PURPLE = (0.6, 0.0, 1.0)
OFF = (0.1, 0.1, 0.1)         # blink off phase, close to the dark groove

FAST = 0.15                   # fast blink: on/off half period [s]
SLOW = 0.5                    # blink: on/off half period [s]

# step -> (color, half period or None for steady)
LOOKS = {
    'patrol': (BLUE, None),
    'returning': (WHITE, None),          # returning to the charger
    'charging': (WHITE, None),
    'report': (RED, FAST),               # 119 call, guardian/resident contact
    'searching': (RED, None),            # resident search
    'to_fire': (RED, None),              # moving to the fire (alarm)
    'guiding': (GREEN, SLOW),
    'suppressing': (PURPLE, SLOW),
    'done': (PURPLE, None),              # contact after suppression
}
ALERT_STAGES = ('report', 'searching', 'to_fire')   # a 119/contact alert shows fast red here


class StatusRing(Node):
    def __init__(self):
        super().__init__('status_ring')
        self.gz = GzNode()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_status', self.on_status, latched)

        self.stage = 'charging'
        self.look = None          # current entry of LOOKS
        self.since = 0.0          # start of the current look (blink phase)
        self.ring = None          # visual entity id
        self.shown = None         # color last sent
        self.finder = self.create_timer(1.0, self.find_ring)
        self.create_timer(0.05, self.tick)

    def on_status(self, msg):
        status = json.loads(msg.data)
        alert = status['alert'] and status['stage'] in ALERT_STAGES
        self.stage = 'report' if alert else status['stage']

    def tick(self):
        if self.ring is None:
            return
        now = time.monotonic()
        look = LOOKS[self.stage]
        if look != self.look:
            self.look, self.since = look, now
            self.get_logger().info(f'Ring: {self.stage}')
        color, half = look
        if half is not None and int((now - self.since) / half) % 2:
            color = OFF
        if color != self.shown:
            self.send(color)

    def send(self, color):
        r, g, b = color
        msg = Visual(id=self.ring)
        for c in (msg.material.ambient, msg.material.diffuse, msg.material.emissive):
            c.r, c.g, c.b, c.a = r, g, b, 1.0
        ok, _ = self.gz.request(WORLD + '/visual_config', msg, Visual, Boolean, 1000)
        if ok:
            self.shown = color

    def find_ring(self):
        # the robot is spawned after this node starts; look up the ring visual id once
        ok, scene = self.gz.request(WORLD + '/scene/info', Empty(), Empty, Scene, 2000)
        if not ok:
            return
        for model in scene.model:
            if model.name != MODEL:
                continue
            for link in model.link:
                for visual in link.visual:
                    if RING in visual.name:
                        self.ring = visual.id
                        self.finder.cancel()
                        return


def main():
    rclpy.init()
    rclpy.spin(StatusRing())
