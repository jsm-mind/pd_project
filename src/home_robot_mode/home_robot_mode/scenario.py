"""Emergency scenarios 1 and 2, step by step (order fixed by the team).

Scenario 1 (mode_event 'fire_detected' in PATROL/CHARGING):
  call_119 + notify_fire -> evacuation -> suppression (drives to /fire_location)
  -> notify_done -> (done_hold s) -> mode_event 'emergency_done'
Scenario 2 (mode_event 'fire_alarm' in PATROL/CHARGING):
  ask_check -> evacuation -> drive to /alarm_location with /fire_check on
  (spin 360 deg there if no fire yet); on /fire_location: call_119 +
  notify_confirmed -> suppression in the alarm room ('here', on arrival)
  -> notify_done -> (done_hold s) -> mode_event 'emergency_done'
  The fire is often seen only from afar (stove fire is hidden by the counter up
  close), where its location estimate is rough, so the alarm room is used.

Requests : /contact_request, /evacuation_request, /suppression_request, /fire_check, /voice
Statuses : /evacuation_status, /suppression_status, /fire_location
"""
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

ACTIVE_MODES = ('PATROL', 'CHARGING')


class Scenario(Node):
    def __init__(self):
        super().__init__('scenario')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.contact = self.create_publisher(String, 'contact_request', 10)
        self.event_pub = self.create_publisher(String, 'mode_event', 10)
        self.evacuation = self.create_publisher(String, 'evacuation_request', 10)
        self.suppression = self.create_publisher(String, 'suppression_request', 10)
        self.fire_check = self.create_publisher(String, 'fire_check', 10)
        self.voice = self.create_publisher(String, 'voice', 10)
        self.create_subscription(String, 'mode', self.on_mode, latched)
        self.create_subscription(String, 'mode_event', self.on_event, 10)
        self.create_subscription(String, 'evacuation_status', self.on_evacuation, 10)
        self.create_subscription(String, 'suppression_status', self.on_suppression, 10)
        self.create_subscription(PoseStamped, 'alarm_location', self.on_alarm, latched)
        self.create_subscription(PoseStamped, 'fire_location', self.on_fire, latched)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin = ActionClient(self, Spin, 'spin')
        # stay put after notify_done so the status ring shows the final step
        self.done_hold = self.declare_parameter('done_hold', 3.0).value
        self.done_timer = None

        self.mode = None
        self.entered = False    # EMERGENCY entered from PATROL/CHARGING
        self.cause = None       # 'fire_detected' | 'fire_alarm'
        self.scenario = None    # 1 | 2
        self.step = None        # 'evacuation' | 'to_alarm' | 'search_fire' | 'stop_spin' | 'suppression' | 'done'
        self.alarm = None
        self.goal_handle = None
        self.fire_found = False

    # ---------- inputs ----------
    def on_mode(self, msg):
        # /mode and /mode_event come from different nodes, so either may arrive first
        prev, self.mode = self.mode, msg.data
        if self.mode == 'EMERGENCY':
            self.entered = self.entered or prev in ACTIVE_MODES
            self.try_start()
        else:
            self.entered = False
            self.cause = None

    def on_alarm(self, msg):
        self.alarm = (msg.pose.position.x, msg.pose.position.y)

    def on_event(self, msg):
        if msg.data not in ('fire_detected', 'fire_alarm') or self.cause is not None:
            return
        if self.mode in ACTIVE_MODES or self.entered:
            self.cause = msg.data
            self.try_start()

    def try_start(self):
        if not self.entered or self.cause is None or self.scenario is not None:
            return
        if self.cause == 'fire_detected':
            self.start(1)
            self.request(self.voice, 'fire_detected')
            self.request(self.contact, 'call_119')
            self.request(self.contact, 'notify_fire')
        else:
            self.start(2)
            self.request(self.voice, 'fire_alarm')
            self.request(self.contact, 'ask_check')
        self.step = 'evacuation'
        self.request(self.evacuation, 'start')

    def on_evacuation(self, msg):
        if self.step != 'evacuation':
            return
        self.log(f'evacuation {msg.data}')
        if self.scenario == 1:
            self.start_suppression()
        else:
            self.step = 'to_alarm'
            self.request(self.fire_check, 'start')
            x, y = self.alarm
            self.log(f'moving to the alarm location ({x:.2f}, {y:.2f})')
            self.go_alarm()

    def on_fire(self, msg):
        if self.step not in ('to_alarm', 'search_fire') or self.fire_found:
            return
        x, y = msg.pose.position.x, msg.pose.position.y
        self.log(f'fire found at ({x:.2f}, {y:.2f})')
        self.fire_found = True
        self.request(self.voice, 'fire_confirmed')
        self.request(self.contact, 'call_119')
        self.request(self.contact, 'notify_confirmed')
        if self.step == 'search_fire':
            # already in the alarm room
            self.step = 'stop_spin'
            self.cancel(then=lambda: self.start_suppression('here'))

    def on_suppression(self, msg):
        if self.step != 'suppression' or msg.data != 'done':
            return
        self.request(self.voice, 'suppression_done')
        self.request(self.contact, 'notify_done')
        self.step = 'done'
        self.done_timer = self.create_timer(self.done_hold, self.finish)

    def finish(self):
        self.done_timer.cancel()
        self.done_timer = None
        self.log('finished')
        self.request(self.event_pub, 'emergency_done')
        self.scenario = None
        self.step = None

    # ---------- steps ----------
    def start(self, scenario):
        self.scenario = scenario
        self.fire_found = False
        self.log('started')

    def start_suppression(self, where='start'):
        self.step = 'suppression'
        self.request(self.suppression, where)

    def go_alarm(self):
        x, y = self.alarm
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        self.send(self.nav, goal, 'to_alarm')

    def on_done(self, step, status):
        if step != self.step:
            return      # canceled after the fire was found
        self.goal_handle = None
        if step == 'to_alarm':
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(f'Alarm location not reached (status {status})')
            if self.fire_found:
                self.start_suppression('here')
                return
            self.step = 'search_fire'
            self.log('no fire seen yet, looking around')
            self.send(self.spin, Spin.Goal(target_yaw=2 * math.pi), 'search_fire')
        elif step == 'search_fire':
            self.get_logger().warn('No fire found at the alarm location')
            self.request(self.fire_check, 'stop')
            self.scenario = None
            self.step = None

    # ---------- helpers ----------
    def send(self, client, goal, step):
        client.wait_for_server()
        client.send_goal_async(goal).add_done_callback(lambda f: self.on_accepted(f, step))

    def on_accepted(self, future, step):
        handle = future.result()
        if not handle.accepted:
            return
        if step != self.step:
            handle.cancel_goal_async()  # step moved on while the goal was being accepted
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self.on_done(step, f.result().status))

    def cancel(self, then):
        # next goal only after the canceled one has ended, or Nav2 aborts it
        if self.goal_handle is not None:
            handle, self.goal_handle = self.goal_handle, None
            handle.cancel_goal_async()
            handle.get_result_async().add_done_callback(lambda _: then())
        else:
            then()

    def request(self, pub, data):
        pub.publish(String(data=data))

    def log(self, text):
        self.get_logger().info(f'[Scenario {self.scenario}] {text}')


def main():
    rclpy.init()
    rclpy.spin(Scenario())
