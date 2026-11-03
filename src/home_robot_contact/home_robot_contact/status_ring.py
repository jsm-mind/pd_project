"""Status ring on the top cover: emissive color follows the current mode/step.

Input : /mode                (std_msgs/String, latched) - PATROL / CHARGING / EMERGENCY
        /projector           (std_msgs/Bool)   - on while guiding the evacuation
        /suppression_status  (std_msgs/String) - 'releasing' while the aerosol is out
Output: material of the ring visual via Gazebo UserCommands visual_config, sent only
        when the color changes (no per-frame cost).
"""
import os
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')  # gz python msgs

import rclpy  # noqa: E402
from gz.msgs10.boolean_pb2 import Boolean  # noqa: E402
from gz.msgs10.empty_pb2 import Empty  # noqa: E402
from gz.msgs10.scene_pb2 import Scene  # noqa: E402
from gz.msgs10.visual_pb2 import Visual  # noqa: E402
from gz.transport13 import Node as GzNode  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

WORLD = '/world/fire_house_world'
MODEL = 'robot'
RING = 'status_ring_visual'   # URDF visual name; lumped name only contains it

COLORS = {
    'patrol': (0.0, 1.0, 0.15),
    'charging': (1.0, 1.0, 1.0),
    'emergency': (1.0, 0.45, 0.0),   # resident search / moving to the fire
    'guiding': (1.0, 0.0, 1.0),
    'suppressing': (0.0, 0.3, 1.0),
}


class StatusRing(Node):
    def __init__(self):
        super().__init__('status_ring')
        self.gz = GzNode()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'mode', self.on_mode, latched)
        self.create_subscription(Bool, 'projector', self.on_projector, 10)
        self.create_subscription(String, 'suppression_status', self.on_suppression, 10)

        self.mode = 'CHARGING'
        self.step = None          # 'guiding' / 'suppressing' inside EMERGENCY
        self.ring = None          # visual entity id
        self.shown = None
        self.finder = self.create_timer(1.0, self.find_ring)

    def on_mode(self, msg):
        self.mode = msg.data
        self.step = None
        self.update()

    def on_projector(self, msg):
        self.step = 'guiding' if msg.data else None
        self.update()

    def on_suppression(self, msg):
        self.step = 'suppressing' if msg.data == 'releasing' else None
        self.update()

    def state(self):
        if self.mode == 'PATROL':
            return 'patrol'
        if self.mode == 'EMERGENCY':
            return self.step or 'emergency'
        return 'charging'

    def update(self):
        state = self.state()
        if self.ring is None or state == self.shown:
            return
        r, g, b = COLORS[state]
        msg = Visual(id=self.ring)
        for c in (msg.material.ambient, msg.material.diffuse, msg.material.emissive):
            c.r, c.g, c.b, c.a = r, g, b, 1.0
        ok, _ = self.gz.request(WORLD + '/visual_config', msg, Visual, Boolean, 1000)
        if ok:
            self.shown = state
            self.get_logger().info(f'Ring: {state}')

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
                        self.update()
                        return


def main():
    rclpy.init()
    rclpy.spin(StatusRing())
