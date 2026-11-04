"""Patrol loop and autonomous return to the charger.

PATROL      : visit waypoints in a loop, spinning 360 deg at each (thermal looks all around).
go_charge   : (event while PATROL) cancel patrol, drive to the charger;
              on arrival, force AMCL updates while stopped (it does not update
              without motion), then publish 'docked' if within dock_tolerance.
EMERGENCY -> CHARGING (suppression done or reset): drive back to the charger and dock.
other modes : cancel any active goal.
auto_start  : publish 'start_patrol' once when Nav2 is up (stand-in for the
              later time-based trigger; keys still work).
"""
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Empty

SETTLE_UPDATES = 10     # AMCL updates while stopped on the pad before judging docked


class Patrol(Node):
    def __init__(self):
        super().__init__('patrol')
        wp = self.declare_parameter('waypoints', [0.0]).value
        self.waypoints = [wp[i:i + 3] for i in range(0, len(wp), 3)]
        self.charger = self.declare_parameter('charger', [0.6, 3.6, 0.0]).value
        self.dock_tol = self.declare_parameter('dock_tolerance', 0.05).value
        self.auto_start = self.declare_parameter('auto_start', True).value

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin = ActionClient(self, Spin, 'spin')
        self.nomotion = self.create_client(Empty, 'request_nomotion_update')
        self.settle_timer = None
        self.settle_left = 0
        self.event_pub = self.create_publisher(String, 'mode_event', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'mode', self.on_mode, latched)
        self.create_subscription(String, 'mode_event', self.on_event, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl, 10)

        self.mode = None
        self.task = None        # 'patrol' | 'return' | None
        self.index = 0
        self.spun = False       # spin done at the current waypoint
        self.goal_handle = None
        self.amcl_xy = None
        if self.auto_start:
            self.start_timer = self.create_timer(1.0, self.on_start_timer)

    # ---------- inputs ----------
    def on_start_timer(self):
        if self.mode is None or not self.nav.server_is_ready():
            return
        self.start_timer.cancel()
        if self.mode == 'CHARGING':
            self.get_logger().info('Auto start: patrol')
            self.event_pub.publish(String(data='start_patrol'))

    def on_mode(self, msg):
        prev, self.mode = self.mode, msg.data
        if self.mode == 'PATROL' and self.task is None:
            self.task = 'patrol'
            self.index = 0
            self.get_logger().info('Patrol start')
            self.send_next()
        elif self.mode == 'CHARGING' and prev == 'EMERGENCY':
            self.get_logger().info('Emergency over, returning to charger')
            self.task = 'return'
            self.cancel(then=self.send_next)
        elif self.mode != 'PATROL':
            self.stop()

    def on_event(self, msg):
        if msg.data == 'go_charge' and self.mode == 'PATROL' and self.task != 'return':
            self.get_logger().info('Returning to charger')
            self.task = 'return'
            self.cancel(then=self.send_next)

    def on_amcl(self, msg):
        p = msg.pose.pose.position
        self.amcl_xy = (p.x, p.y)

    # ---------- navigation ----------
    def send_next(self):
        if self.task == 'patrol':
            self.spun = False
            x, y, yaw = self.waypoints[self.index]
            self.get_logger().info(f'Waypoint {self.index}: ({x}, {y})')
        elif self.task == 'return':
            x, y, yaw = self.charger
        else:
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.nav.wait_for_server()
        task = self.task
        self.nav.send_goal_async(goal).add_done_callback(
            lambda f: self.on_accepted(f, task))

    def on_accepted(self, future, task):
        handle = future.result()
        if task != self.task:
            return
        if not handle.accepted:
            # Nav2 action server exists before it is activated; retry until it accepts
            self.get_logger().warn('Goal rejected (Nav2 not active yet?), retrying in 2 s')
            retry = self.create_timer(2.0, lambda: (retry.cancel(), self.send_next()))
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, task))

    def on_result(self, future, task):
        if task != self.task or future.result().status == GoalStatus.STATUS_CANCELED:
            return
        self.goal_handle = None
        if task == 'patrol':
            if not self.spun:
                self.spun = True
                self.spin.wait_for_server()
                self.spin.send_goal_async(Spin.Goal(target_yaw=2 * math.pi)).add_done_callback(
                    lambda f: self.on_accepted(f, task))
                return
            self.index = (self.index + 1) % len(self.waypoints)
            self.send_next()
        else:
            self.settle_left = SETTLE_UPDATES
            self.settle_timer = self.create_timer(0.3, self.on_settle)

    def on_settle(self):
        if self.task != 'return':
            self.settle_timer.cancel()
            return
        if self.settle_left > 0:
            self.nomotion.call_async(Empty.Request())
            self.settle_left -= 1
            return
        self.settle_timer.cancel()
        if self.is_docked():
            self.get_logger().info('Docked on charger')
            self.task = None
            if self.mode == 'PATROL':   # already CHARGING after an emergency
                self.event_pub.publish(String(data='docked'))
        else:
            self.get_logger().info('Not on pad yet, retrying')
            self.send_next()

    def is_docked(self):
        if self.amcl_xy is None:
            return False
        d = math.hypot(self.amcl_xy[0] - self.charger[0], self.amcl_xy[1] - self.charger[1])
        self.get_logger().info(f'AMCL distance to pad: {d:.3f} m')
        return d <= self.dock_tol

    def cancel(self, then=None):
        # send the next goal only after the cancel is done, so they do not race
        if self.goal_handle is not None:
            future = self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            if then is not None:
                future.add_done_callback(lambda _: then())
        elif then is not None:
            then()

    def stop(self):
        if self.task is not None:
            self.get_logger().info(f'Stop ({self.mode})')
        self.task = None
        self.cancel()


def main():
    rclpy.init()
    rclpy.spin(Patrol())
