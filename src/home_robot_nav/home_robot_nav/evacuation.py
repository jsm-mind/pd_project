"""Resident search and evacuation guidance.

/evacuation_request 'start':
  SEARCH   visit search points (nearest first), spin 360 deg at each,
           until /person_location arrives
  APPROACH drive to approach_dist in front of the resident (skipped if closer)
  GUIDE    voice + floor arrow on, resident follows, drive to the exit at
           guide_speed (Nav2 speed limit; search runs at full speed)
  done     voice, arrow off, /evacuation_status 'done'
           ('not_found' if every search point was scanned without a person,
            'failed' if the exit could not be reached in 4 tries)
"""
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

# voice line keys (home_robot_contact voice_lines)
SAY_FOUND = 'evac_found'
SAY_GUIDE = 'evac_guide'
SAY_DONE = 'evac_done'


class Evacuation(Node):
    def __init__(self):
        super().__init__('evacuation')
        pts = self.declare_parameter('search_points', [0.0, 0.0]).value
        self.points = [pts[i:i + 2] for i in range(0, len(pts), 2)]
        self.exit = self.declare_parameter('exit', [4.2, 0.7, -1.57]).value
        self.approach_dist = self.declare_parameter('approach_dist', 1.0).value
        self.voice_period = self.declare_parameter('voice_period', 8.0).value
        self.guide_speed = self.declare_parameter('guide_speed', 0.8).value

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin = ActionClient(self, Spin, 'spin')
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.voice = self.create_publisher(String, 'voice', 10)
        self.projector = self.create_publisher(Bool, 'projector', 10)
        self.follow = self.create_publisher(Bool, 'resident_follow', 10)
        self.status = self.create_publisher(String, 'evacuation_status', 10)
        self.speed_limit = self.create_publisher(SpeedLimit, 'speed_limit', 10)
        self.create_subscription(String, 'evacuation_request', self.on_request, 10)
        self.create_subscription(PointStamped, 'person_location', self.on_person, 10)

        self.state = None       # 'search' | 'approach' | 'guide' | None
        self.queue = []
        self.guide_tries = 0
        self.goal_handle = None
        self.voice_timer = None

    # ---------- inputs ----------
    def on_request(self, msg):
        if msg.data != 'start' or self.state is not None:
            return
        robot = self.robot_xy()
        self.queue = sorted(self.points, key=lambda p: math.dist(p, robot)) if robot else list(self.points)
        self.get_logger().info('Searching for the resident')
        self.state = 'search'
        self.next_point()

    def on_person(self, msg):
        if self.state != 'search':
            return
        person = (msg.point.x, msg.point.y)
        robot = self.robot_xy()
        if robot is None:
            return
        self.get_logger().info(f'Resident found at ({person[0]:.2f}, {person[1]:.2f})')
        d = math.dist(person, robot)
        if d <= self.approach_dist:
            self.state = 'approach'
            self.cancel(then=self.start_guide)
            return
        self.state = 'approach'
        k = (d - self.approach_dist) / d
        x = robot[0] + (person[0] - robot[0]) * k
        y = robot[1] + (person[1] - robot[1]) * k
        yaw = math.atan2(person[1] - robot[1], person[0] - robot[0])
        self.cancel(then=lambda: self.go(NavigateToPose, (x, y, yaw), 'approach'))

    # ---------- steps ----------
    def next_point(self):
        if not self.queue:
            self.get_logger().warn('Resident not found at any search point')
            self.finish('not_found')
            return
        x, y = self.queue.pop(0)
        self.get_logger().info(f'Search point ({x}, {y})')
        self.go(NavigateToPose, (x, y, None), 'search')

    def start_guide(self):
        self.get_logger().info('Guiding the resident to the exit')
        self.state = 'guide'
        self.guide_tries = 0
        self.say(SAY_FOUND)
        self.set_speed(self.guide_speed)
        self.projector.publish(Bool(data=True))
        self.follow.publish(Bool(data=True))
        self.voice_timer = self.create_timer(self.voice_period, lambda: self.say(SAY_GUIDE))
        self.go(NavigateToPose, tuple(self.exit), 'guide')

    def finish(self, result):
        if result == 'done':
            self.say(SAY_DONE)
        if self.voice_timer is not None:
            self.voice_timer.cancel()
            self.voice_timer = None
        self.set_speed(0.0)
        self.projector.publish(Bool(data=False))
        self.follow.publish(Bool(data=False))
        self.state = None
        self.status.publish(String(data=result))
        self.get_logger().info(f'Evacuation {result}')

    def on_done(self, kind, step, status):
        if step != self.state:
            return      # result of a goal that was replaced
        self.goal_handle = None
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'{kind.__name__} for {step} ended with status {status}')
        if step == 'search':
            if kind is NavigateToPose:
                self.go(Spin, None, 'search')
            else:
                self.next_point()
        elif step == 'approach':
            self.start_guide()
        elif step == 'guide':
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.finish('done')
            elif self.guide_tries < 3:
                self.guide_tries += 1
                self.go(NavigateToPose, tuple(self.exit), 'guide')
            else:
                self.finish('failed')

    # ---------- helpers ----------
    def go(self, kind, pose, step):
        if kind is Spin:
            client, goal = self.spin, Spin.Goal(target_yaw=2 * math.pi)
        else:
            x, y, yaw = pose
            if yaw is None:
                robot = self.robot_xy() or (x, y)
                yaw = math.atan2(y - robot[1], x - robot[0])
            client, goal = self.nav, NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.pose.position.x = float(x)
            goal.pose.pose.position.y = float(y)
            goal.pose.pose.orientation.z = math.sin(yaw / 2)
            goal.pose.pose.orientation.w = math.cos(yaw / 2)
        client.wait_for_server()
        client.send_goal_async(goal).add_done_callback(
            lambda f: self.on_accepted(f, kind, step))

    def on_accepted(self, future, kind, step):
        handle = future.result()
        if not handle.accepted or step != self.state:
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self.on_done(kind, step, f.result().status))

    def cancel(self, then):
        # next goal only after the canceled one has ended, or Nav2 aborts it
        if self.goal_handle is not None:
            handle, self.goal_handle = self.goal_handle, None
            handle.cancel_goal_async()
            handle.get_result_async().add_done_callback(lambda _: then())
        else:
            then()

    def set_speed(self, limit):
        # absolute [m/s]; 0 removes the limit
        self.speed_limit.publish(SpeedLimit(percentage=False, speed_limit=float(limit)))

    def say(self, text):
        self.voice.publish(String(data=text))

    def robot_xy(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time(), Duration(seconds=0.2))
        except Exception:  # noqa: BLE001 - TF not ready yet
            return None
        return (t.transform.translation.x, t.transform.translation.y)


def main():
    rclpy.init()
    rclpy.spin(Evacuation())
