"""Fire suppression with the one-shot solid aerosol unit (no aiming).

/suppression_request 'start':
  drive to standoff from /fire_location (along the line robot -> fire),
  ignite the aerosol (terminal), show the cloud (aerosol model spawned at
  aerosol_outlet_link, visual only), remove the fire model after fire_out_delay and the
  cloud after cloud_time; /suppression_status 'releasing' at ignition, 'done' at the end.
  If the standoff point is not reached (e.g. inside the counter's costmap), release where
  the robot stopped when within release_radius of the fire, else retry (4 tries in all).
/suppression_request 'here': release at the current position (robot already in the
  fire space, scenario 2: alarm room).
The unit is single-use: later requests are only logged.
"""
import os
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')  # gz python msgs

import math  # noqa: E402

import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from gz.msgs10.boolean_pb2 import Boolean  # noqa: E402
from gz.msgs10.entity_factory_pb2 import EntityFactory  # noqa: E402
from gz.msgs10.entity_pb2 import Entity  # noqa: E402
from gz.transport13 import Node as GzNode  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from rclpy.time import Time  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

WORLD = '/world/fire_house_world'


class Suppression(Node):
    def __init__(self):
        super().__init__('suppression')
        self.standoff = self.declare_parameter('standoff', 0.3).value
        self.fire_out_delay = self.declare_parameter('fire_out_delay', 5.0).value
        self.cloud_time = self.declare_parameter('cloud_time', 20.0).value
        self.release_radius = self.declare_parameter('release_radius', 1.5).value

        self.gz = GzNode()
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.status = self.create_publisher(String, 'suppression_status', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, 'fire_location', self.on_fire, latched)
        self.create_subscription(String, 'suppression_request', self.on_request, 10)

        self.fire = None
        self.used = False
        self.busy = False
        self.tries = 0
        self.goal = None

    def on_fire(self, msg):
        self.fire = (msg.pose.position.x, msg.pose.position.y)

    def on_request(self, msg):
        if msg.data not in ('start', 'here') or self.busy:
            return
        if self.used:
            self.get_logger().warn('Aerosol unit already used (single-use)')
            return
        if msg.data == 'here':
            self.busy = True
            self.release()
            return
        robot = self.robot_xy()
        if self.fire is None or robot is None:
            self.get_logger().warn('No fire location or robot pose yet')
            return
        d = math.dist(self.fire, robot)
        k = max(d - self.standoff, 0.0) / d if d > 1e-3 else 0.0
        yaw = math.atan2(self.fire[1] - robot[1], self.fire[0] - robot[0])
        self.goal = (robot[0] + (self.fire[0] - robot[0]) * k,
                     robot[1] + (self.fire[1] - robot[1]) * k, yaw)
        self.get_logger().info(
            f'Moving to the fire space ({self.goal[0]:.2f}, {self.goal[1]:.2f})')
        self.busy = True
        self.tries = 0
        self.send_goal()

    def send_goal(self):
        x, y, yaw = self.goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.nav.wait_for_server()
        self.nav.send_goal_async(goal).add_done_callback(self.on_accepted)

    def on_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.on_result(GoalStatus.STATUS_ABORTED)
            return
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f.result().status))

    def on_result(self, status):
        robot = self.robot_xy()
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.release()
        elif robot is not None and math.dist(robot, self.fire) <= self.release_radius:
            # already in the fire space; the aerosol fills it, no need to get closer
            self.get_logger().warn('Standoff point unreachable; in the fire space, releasing here')
            self.release()
        elif self.tries < 3:
            self.tries += 1
            self.get_logger().warn(f'Could not reach the fire space (status {status}), retrying')
            self.send_goal()
        else:
            # goal may sit inside the furniture (location error); the robot is
            # as close as it gets, so release here
            self.get_logger().warn('Standoff point unreachable; releasing where the robot stopped')
            self.release()

    def release(self):
        self.used = True
        bar = '=' * 60
        print(f'{bar}\n[에어로졸] 전기 점화 - 고체 에어로졸 방출\n{bar}', flush=True)
        self.status.publish(String(data='releasing'))
        # cloud model frame = outlet frame (emits along +z)
        t = self.lookup('aerosol_outlet_link')
        req = EntityFactory(sdf_filename='model://aerosol', name='aerosol')
        if t is not None:
            p, q = t.transform.translation, t.transform.rotation
            req.pose.position.x, req.pose.position.y, req.pose.position.z = p.x, p.y, p.z
            req.pose.orientation.x, req.pose.orientation.y = q.x, q.y
            req.pose.orientation.z, req.pose.orientation.w = q.z, q.w
        self.gz.request(WORLD + '/create', req, EntityFactory, Boolean, 2000)
        self.once(self.fire_out_delay, self.fire_out)
        self.once(self.cloud_time, self.cloud_gone)

    def fire_out(self):
        self.remove('fire')
        self.get_logger().info('Fire extinguished')

    def cloud_gone(self):
        self.remove('aerosol')
        self.busy = False
        self.status.publish(String(data='done'))
        self.get_logger().info('Suppression done')

    # ---------- helpers ----------
    def once(self, delay, fn):
        def fire_once():
            timer.cancel()
            fn()
        timer = self.create_timer(delay, fire_once)

    def remove(self, name):
        self.gz.request(WORLD + '/remove', Entity(name=name, type=Entity.MODEL),
                        Entity, Boolean, 2000)

    def lookup(self, frame):
        try:
            return self.tf.lookup_transform('map', frame, Time(), Duration(seconds=0.2))
        except Exception:  # noqa: BLE001 - TF not ready yet
            return None

    def robot_xy(self):
        t = self.lookup('base_footprint')
        return None if t is None else (t.transform.translation.x, t.transform.translation.y)


def main():
    rclpy.init()
    rclpy.spin(Suppression())
