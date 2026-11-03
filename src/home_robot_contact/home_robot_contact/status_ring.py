"""Status ring on the top cover: emissive color and blinking follow the current mode/step.

Input : /mode                (std_msgs/String, latched) - PATROL / CHARGING / EMERGENCY
        /mode_event          (std_msgs/String) - 'go_charge' (returning to the charger)
        /contact_request     (std_msgs/String) - 119 / contact step, 'notify_done'
        /projector           (std_msgs/Bool)   - on while guiding the evacuation
        /suppression_status  (std_msgs/String) - 'releasing' while the aerosol is out
Output: material of the ring visual via Gazebo UserCommands visual_config, sent only
        when the shown color changes (blink = one request per toggle, wall clock).
"""
import os
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')  # gz python msgs

import time  # noqa: E402

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

BLUE = (0.0, 0.35, 1.0)
WHITE = (1.0, 1.0, 1.0)
RED = (1.0, 0.0, 0.0)
GREEN = (0.1, 1.0, 0.2)       # same as the floor arrow
PURPLE = (0.6, 0.0, 1.0)
OFF = (0.1, 0.1, 0.1)         # blink off phase, close to the dark groove

FAST = 0.15                   # fast blink: on/off half period [s]
SLOW = 0.5                    # blink: on/off half period [s]
ALERT_TIME = 3.0              # fast red blink after the emergency starts / a 119 contact [s]
ALERT_REQUESTS = ('call_119', 'notify_fire', 'ask_check', 'notify_confirmed')

# step -> (color, half period or None for steady)
LOOKS = {
    'patrol': (BLUE, None),
    'charging': (WHITE, None),           # returning to the charger / charging
    'alert': (RED, FAST),                # 119 call, guardian/resident contact
    'emergency': (RED, None),            # resident search / moving to the fire (alarm)
    'guiding': (GREEN, SLOW),
    'suppressing': (PURPLE, SLOW),
    'done': (PURPLE, None),              # contact after suppression
}


class StatusRing(Node):
    def __init__(self):
        super().__init__('status_ring')
        self.gz = GzNode()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'mode', self.on_mode, latched)
        self.create_subscription(String, 'mode_event', self.on_event, 10)
        self.create_subscription(String, 'contact_request', self.on_contact, 10)
        self.create_subscription(Bool, 'projector', self.on_projector, 10)
        self.create_subscription(String, 'suppression_status', self.on_suppression, 10)

        self.mode = 'CHARGING'
        self.returning = False    # PATROL, driving back to the charger
        self.step = None          # 'guiding' / 'suppressing' / 'done' inside EMERGENCY
        self.alert_until = 0.0
        self.look = None          # current entry of LOOKS
        self.since = 0.0          # start of the current look (blink phase)
        self.ring = None          # visual entity id
        self.shown = None         # color last sent
        self.finder = self.create_timer(1.0, self.find_ring)
        self.create_timer(0.05, self.tick)

    def on_mode(self, msg):
        if msg.data == self.mode:
            return
        self.mode = msg.data
        self.returning = False
        self.step = None
        if self.mode == 'EMERGENCY':
            self.alert_until = time.monotonic() + ALERT_TIME

    def on_event(self, msg):
        if msg.data == 'go_charge' and self.mode == 'PATROL':
            self.returning = True

    def on_contact(self, msg):
        if msg.data in ALERT_REQUESTS:
            self.alert_until = time.monotonic() + ALERT_TIME
        elif msg.data == 'notify_done':
            self.step = 'done'

    def on_projector(self, msg):
        self.step = 'guiding' if msg.data else None

    def on_suppression(self, msg):
        if msg.data == 'releasing':
            self.step = 'suppressing'

    def state(self):
        if self.mode == 'PATROL':
            return 'charging' if self.returning else 'patrol'
        if self.mode == 'EMERGENCY':
            if self.step:
                return self.step
            return 'alert' if time.monotonic() < self.alert_until else 'emergency'
        return 'charging'

    def tick(self):
        if self.ring is None:
            return
        now = time.monotonic()
        look = LOOKS[self.state()]
        if look != self.look:
            self.look, self.since = look, now
            self.get_logger().info(f'Ring: {self.state()}')
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
