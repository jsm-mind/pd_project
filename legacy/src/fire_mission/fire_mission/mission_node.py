"""Fire response state machine.

PATROL --(alarm on)--> ALARM_SWEEP --(one round, no fire)--> alarm on:  report, sweep again
                                                             alarm off: notify owner, PATROL
PATROL / ALARM_SWEEP -> FIRE_CONFIRMED (report) -> APPROACH -> ALIGN -> SPRAY -> VERIFY
                                                                  |- fire gone   -> PATROL
                                                                  |- fire remains -> APPROACH

Uses only Nav2, /cmd_vel, /scan, the saved map (/map), AMCL pose (TF map -> base_link),
the thermal detector and the alarm heard by the robot (/alarm/active, no position).
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, Float32
from tf2_ros import Buffer, TransformListener

from fire_mission.reporter import Reporter

LOOP_PERIOD = 0.1          # [s]
BEARING_FRESH = 0.3        # [s] a bearing older than this is stale
RANGE_WINDOW = math.radians(2.0)  # a fire 0.3 m wide spans about +-2 deg at 4 m
STATIC_MARGIN = 0.15       # [m] LiDAR hits this close to a mapped obstacle are not the fire
COARSE_TOLERANCE = math.radians(10.0)
ALIGN_STABLE_FRAMES = 3


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, limit):
    return max(-limit, min(limit, v))


class Mission:

    def __init__(self, nav):
        self.nav = nav
        p = nav.declare_parameter
        flat = p('waypoints', [0.0, 0.0]).value
        self.waypoints = list(zip(flat[0::2], flat[1::2]))
        self.spin_time = p('spin_time_allowance', 30.0).value
        self.room_names = p('room_names', ['']).value
        flat = p('room_bounds', [0.0, 0.0, 0.0, 0.0]).value
        self.room_bounds = [flat[i:i + 4] for i in range(0, len(flat), 4)]
        self.spray_distance = p('spray_distance', 1.5).value
        self.min_distance = p('min_distance', 1.0).value
        self.max_distance = p('max_distance', 2.0).value
        self.fire_radius = p('fire_radius', 0.15).value
        self.align_tol = math.radians(p('align_tolerance_deg', 3.0).value)
        self.realign = math.radians(p('realign_deg', 8.0).value)
        self.align_gain = p('align_gain', 1.5).value
        self.max_align_speed = p('max_align_speed', 0.4).value
        self.align_timeout = p('align_timeout', 20.0).value
        self.spray_duration = p('spray_duration', 12.0).value
        self.scan_speed = p('scan_speed', 0.6).value
        self.verify_timeout = p('verify_timeout', 30.0).value
        self.max_retries = p('max_retries', 3).value
        self.reporter = Reporter(nav.get_logger(),
                                 p('report_config', '~/.config/fire_robot/report.yaml').value)

        self.detected = False
        self.alarm = False
        self.alarm_rise = False   # alarm turned on during PATROL, not yet handled
        self.bearing = None
        self.bearing_time = None
        self.scan = None
        self.static_map = None  # (occupied grid, resolution, origin x, origin y)
        nav.create_subscription(Bool, '/fire_detection/detected', self.on_detected, 10)
        nav.create_subscription(Float32, '/fire_detection/bearing', self.on_bearing, 10)
        nav.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.cmd_pub = nav.create_publisher(Twist, '/cmd_vel', 10)
        self.spray_pub = nav.create_publisher(Bool, '/spray/active', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        nav.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        nav.create_subscription(Bool, '/alarm/active', self.on_alarm, latched)
        self.started_pub = nav.create_publisher(Empty, '/patrol/started', latched)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, nav)
        # Sensor (x, y) in base_link, read from the robot description's static TF in run()
        self.thermal_xy = None
        self.lidar_xy = None

        # PATROL progress
        self.wp_index = 0
        self.wp_phase = None   # 'goto' | 'scan'
        self.round_start = None
        self.patrol_round = 1
        # Current fire event
        self.fire = None       # estimated (x, y) in map
        self.reported = False
        self.retries = 0
        self.detect_time = None
        self.state = None
        self.state_time = None
        self.sub = None        # per-state scratch data

    # ---------------------------------------------------------------- inputs
    def on_detected(self, msg):
        self.detected = msg.data

    def on_alarm(self, msg):
        # Only the off -> on edge during PATROL starts a sweep: while fighting a fire the
        # alarm is ignored, and it is usually still on right after the fire is out.
        if msg.data and not self.alarm and self.state == 'PATROL':
            self.alarm_rise = True
        self.alarm = msg.data

    def on_bearing(self, msg):
        self.bearing = msg.data
        self.bearing_time = self.now()

    def on_scan(self, msg):
        self.scan = msg

    def on_map(self, msg):
        info = msg.info
        grid = np.array(msg.data, dtype=np.int8).reshape(info.height, info.width)
        # Unknown cells (outside the house) count as obstacles too
        self.static_map = (grid != 0, info.resolution,
                           info.origin.position.x, info.origin.position.y)

    def now(self):
        return self.nav.get_clock().now()

    def elapsed(self, since):
        return (self.now() - since).nanoseconds * 1e-9

    def fresh_bearing(self):
        if self.bearing_time is None or self.elapsed(self.bearing_time) > BEARING_FRESH:
            return None
        return self.bearing

    def robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time(), Duration(seconds=0.2))
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def sensor_origin(self, pose, xy):
        """Map position of a sensor mounted at xy in base_link, for robot pose (x, y, yaw)."""
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return x + xy[0] * c - xy[1] * s, y + xy[0] * s + xy[1] * c

    def is_static(self, x, y):
        """True if (x, y) in map is within STATIC_MARGIN of a mapped obstacle."""
        occupied, res, ox, oy = self.static_map
        h, w = occupied.shape
        c, r, k = int((x - ox) / res), int((y - oy) / res), int(math.ceil(STATIC_MARGIN / res))
        if not (0 <= c < w and 0 <= r < h):
            return True
        return bool(occupied[max(r - k, 0):r + k + 1, max(c - k, 0):c + k + 1].any())

    def range_at(self, bearing):
        """Distance from the thermal camera to the closest LiDAR hit within RANGE_WINDOW of
        bearing (rad, thermal camera frame), or None.

        The LiDAR and the camera may sit at different points, so each hit's bearing is
        taken from the camera. Beams hitting walls or furniture of the saved map are
        skipped: the fire is not in the map, while a door frame next to the line of sight
        would give a short range.
        """
        s = self.scan
        pose = self.robot_pose()
        if s is None or pose is None:
            return None
        yaw = pose[2]
        cx, cy = self.sensor_origin(pose, self.thermal_xy)
        lx, ly = self.sensor_origin(pose, self.lidar_xy)
        best = None
        for i, r in enumerate(s.ranges):
            if not s.range_min <= r <= s.range_max:
                continue
            a = yaw + s.angle_min + i * s.angle_increment
            hx, hy = lx + r * math.cos(a), ly + r * math.sin(a)
            d = math.hypot(hx - cx, hy - cy)
            if best is not None and d >= best:
                continue
            if abs(wrap(math.atan2(hy - cy, hx - cx) - yaw - bearing)) > RANGE_WINDOW:
                continue
            if self.static_map is not None and self.is_static(hx, hy):
                continue
            best = d
        return best

    def estimate_fire(self, bearing):
        """Fire center in map from robot pose + bearing + LiDAR range; None if unavailable."""
        pose = self.robot_pose()
        r = self.range_at(bearing)
        if pose is None or r is None:
            return None
        yaw = pose[2]
        cx, cy = self.sensor_origin(pose, self.thermal_xy)
        dist = r + self.fire_radius
        return cx + dist * math.cos(yaw + bearing), cy + dist * math.sin(yaw + bearing)

    def room_of(self, x, y):
        for name, (x0, y0, x1, y1) in zip(self.room_names, self.room_bounds):
            if x0 <= x <= x1 and y0 <= y <= y1:
                return name
        return '알 수 없음'

    def fire_location(self):
        """Report text for the estimated fire position, or None."""
        if not self.fire:
            return None
        fx, fy = self.fire
        return f'{self.room_of(fx, fy)} ({fx:.2f}, {fy:.2f})'

    # --------------------------------------------------------------- outputs
    def drive(self, w=0.0):
        cmd = Twist()
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def set_spray(self, on):
        self.spray_pub.publish(Bool(data=on))

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def task_result(self):
        """None while the Nav2 task is running, else its TaskResult."""
        if not self.nav.isTaskComplete():
            return None
        return self.nav.getResult()

    # --------------------------------------------------------- state control
    def enter(self, state, **sub):
        if state != self.state:
            self.nav.info(f'State: {self.state} -> {state}')
        self.state = state
        self.state_time = self.now()
        self.sub = sub
        getattr(self, 'enter_' + state.lower(), lambda: None)()

    def step(self):
        getattr(self, 'step_' + self.state.lower())()

    # ---------------------------------------------------------------- PATROL
    def enter_patrol(self):
        self.goto_waypoint()

    def goto_waypoint(self):
        if not self.nav.nav_to_pose_client.server_is_ready():
            self.wp_phase = 'wait'   # retried from step_patrol
            return
        x, y = self.waypoints[self.wp_index]
        px, py = self.waypoints[self.wp_index - 1]
        self.wp_phase = 'goto'
        self.nav.goToPose(self.make_pose(x, y, math.atan2(y - py, x - px)))

    def next_waypoint(self):
        n = len(self.waypoints)
        self.wp_index = (self.wp_index + 1) % n
        if self.wp_index == 0:
            self.nav.info(f'Patrol round {self.patrol_round} done in '
                          f'{self.elapsed(self.round_start):.0f} s')
            self.patrol_round += 1
            self.round_start = self.now()
        self.goto_waypoint()

    def step_waypoint(self):
        """Go to the current waypoint, then scan 360 deg there.
        Returns True once done with it (scanned, or skipped on failure)."""
        if self.wp_phase == 'wait':
            self.goto_waypoint()
            return False
        result = self.task_result()
        if result is None:
            return False
        n = len(self.waypoints)
        if self.wp_phase == 'goto':
            if result != TaskResult.SUCCEEDED:
                self.nav.warn(f'Waypoint {self.wp_index + 1}/{n} not reached, skipping')
                return True
            self.nav.info(f'Waypoint {self.wp_index + 1}/{n} reached, scanning')
            if not self.nav.spin_client.server_is_ready():
                self.nav.warn('Spin server not ready, skipping scan')
                return True
            self.wp_phase = 'scan'
            self.nav.spin(spin_dist=2.0 * math.pi, time_allowance=int(self.spin_time))
            return False
        if result != TaskResult.SUCCEEDED:
            self.nav.warn(f'Scan at waypoint {self.wp_index + 1}/{n} failed')
        else:
            self.nav.info(f'Scan at waypoint {self.wp_index + 1}/{n} done')
        return True

    def stop_and_enter(self, state):
        self.nav.cancelTask()
        self.drive(0.0)
        self.enter(state)

    def step_patrol(self):
        if self.detected:
            self.alarm_rise = False
            self.stop_and_enter('FIRE_CONFIRMED')
            return
        if self.alarm_rise:
            self.alarm_rise = False
            self.nav.warn('Alarm heard, starting the alarm sweep')
            self.stop_and_enter('ALARM_SWEEP')
            return
        if self.step_waypoint():
            self.next_waypoint()

    # ----------------------------------------------------------- ALARM_SWEEP
    def nearest_waypoint(self):
        pose = self.robot_pose()
        if pose is None:
            return self.wp_index
        return min(range(len(self.waypoints)),
                   key=lambda i: math.hypot(self.waypoints[i][0] - pose[0],
                                            self.waypoints[i][1] - pose[1]))

    def enter_alarm_sweep(self):
        # All waypoints once, in patrol order, starting from the nearest one
        self.wp_index = self.nearest_waypoint()
        self.sub['left'] = len(self.waypoints)
        self.sub['round'] = 1
        self.nav.info(f'Alarm sweep round 1 from waypoint {self.wp_index + 1}')
        self.goto_waypoint()

    def step_alarm_sweep(self):
        if self.detected:
            self.stop_and_enter('FIRE_CONFIRMED')
            return
        if not self.step_waypoint():
            return
        self.wp_index = (self.wp_index + 1) % len(self.waypoints)
        self.sub['left'] -= 1
        if self.sub['left'] > 0:
            self.goto_waypoint()
            return
        if self.alarm:
            self.nav.warn(f'Alarm sweep round {self.sub["round"]} found no fire, alarm still on')
            self.reporter.report('ALARM_NO_FIRE')
            self.sub['left'] = len(self.waypoints)
            self.sub['round'] += 1
            self.nav.info(f'Alarm sweep round {self.sub["round"]} from waypoint '
                          f'{self.wp_index + 1}')
            self.goto_waypoint()
            return
        self.nav.info('Alarm sweep found no fire and the alarm is off, resuming patrol')
        self.reporter.report('OWNER_NOTICE')
        self.enter('PATROL')

    # -------------------------------------------------------- FIRE_CONFIRMED
    def enter_fire_confirmed(self):
        if not self.reported:
            # New fire event; re-detections from VERIFY keep the retry count and timer
            self.detect_time = self.now()
            self.retries = 0
        self.fire = None

    def step_fire_confirmed(self):
        # Let the robot settle, then estimate the fire from a fresh (stationary) bearing
        self.drive(0.0)
        b = self.fresh_bearing()
        waited = self.elapsed(self.state_time)
        if waited < 0.5 or (b is None and waited < 2.0):
            return
        if b is None:
            b = self.bearing
        self.fire = self.estimate_fire(b)
        if self.fire is None:
            self.nav.warn('Fire position unknown (no LiDAR range), aligning by bearing only')
        if not self.reported:
            self.reporter.report('FIRE_CONFIRMED', self.fire_location() or '알 수 없음')
            self.reported = True
        self.enter('APPROACH' if self.fire else 'ALIGN')

    # -------------------------------------------------------------- APPROACH
    def enter_approach(self):
        if not self.nav.nav_to_pose_client.server_is_ready():
            self.sub['wait'] = True   # retried from step_approach
            return
        pose = self.robot_pose()
        fx, fy = self.fire
        if pose is not None:
            d = math.hypot(fx - pose[0], fy - pose[1])
            if self.min_distance + 0.2 <= d <= self.max_distance - 0.2:
                self.sub['skip'] = True
                return
            ux, uy = (pose[0] - fx) / d, (pose[1] - fy) / d
        else:
            ux, uy = 1.0, 0.0
        gx, gy = fx + self.spray_distance * ux, fy + self.spray_distance * uy
        self.nav.info(f'Approaching fire at ({fx:.2f}, {fy:.2f}) via ({gx:.2f}, {gy:.2f})')
        self.nav.goToPose(self.make_pose(gx, gy, math.atan2(fy - gy, fx - gx)))

    def step_approach(self):
        if self.sub.get('wait'):
            self.sub = {}
            self.enter_approach()
            return
        if self.sub.get('skip'):
            self.enter('ALIGN')
            return
        result = self.task_result()
        if result is None:
            return
        if result != TaskResult.SUCCEEDED:
            self.nav.warn('Approach goal not reached, aligning from here')
        self.enter('ALIGN')

    # ----------------------------------------------------------------- ALIGN
    def heading_error_to_fire(self):
        pose = self.robot_pose()
        if pose is None or self.fire is None:
            return None
        cx, cy = self.sensor_origin(pose, self.thermal_xy)
        return wrap(math.atan2(self.fire[1] - cy, self.fire[0] - cx) - pose[2])

    def step_align(self):
        if self.elapsed(self.state_time) > self.align_timeout:
            self.nav.warn('Align timeout')
            self.drive(0.0)
            self.enter('VERIFY')
            return
        b = self.fresh_bearing()
        if b is None:
            # Coarse: turn toward the estimated fire position until the camera sees it
            err = self.heading_error_to_fire()
            if err is None:
                self.drive(self.max_align_speed)   # unknown position: search by turning
            elif abs(err) > COARSE_TOLERANCE or not self.detected:
                self.drive(clamp(self.align_gain * err, self.max_align_speed))
            else:
                self.drive(0.0)
            self.sub['stable'] = 0
            return
        # Fine: stationary thermal bearing
        if abs(b) > self.align_tol:
            self.drive(clamp(self.align_gain * b, self.max_align_speed))
            self.sub['stable'] = 0
            return
        self.drive(0.0)
        self.sub['stable'] = self.sub.get('stable', 0) + 1
        if self.sub['stable'] < ALIGN_STABLE_FRAMES:
            return
        if self.sub.get('realign'):
            # Rotating in place keeps the distance checked before spraying; the LiDAR may
            # still see spray particles here, so it is not used.
            self.nav.info(f'Realigned: bearing {math.degrees(b):.1f} deg')
            self.enter('SPRAY')
            return
        est = self.estimate_fire(b)
        pose = self.robot_pose()
        dist = None
        if est is not None:
            self.fire = est
            if pose is not None:
                dist = math.hypot(est[0] - pose[0], est[1] - pose[1])
        if dist is not None and self.min_distance <= dist <= self.max_distance:
            self.nav.info(f'Aligned: bearing {math.degrees(b):.1f} deg, distance {dist:.2f} m')
            self.enter('SPRAY')
            return
        self.nav.warn(f'Aligned but distance {dist} out of range, approaching again')
        self.retry()

    def retry(self):
        self.retries += 1
        if self.retries > self.max_retries or self.fire is None:
            self.nav.error('Giving up on this fire, resuming patrol')
            self.finish_fire()
            return
        self.enter('APPROACH')

    # ----------------------------------------------------------------- SPRAY
    def enter_spray(self):
        self.drive(0.0)
        self.nav.info(f'Spraying ({self.elapsed(self.detect_time):.1f} s after detection)')

    def step_spray(self):
        # Spray for a fixed burst: the particles can hide the fire from the thermal camera,
        # so extinguishing is checked afterwards in VERIFY.
        if self.elapsed(self.state_time) >= self.spray_duration:
            self.set_spray(False)
            self.nav.info('Spray burst done')
            self.enter('VERIFY')
            return
        b = self.fresh_bearing()
        if b is not None and abs(b) > self.realign:
            self.set_spray(False)
            self.nav.info(f'Bearing drifted to {math.degrees(b):.1f} deg, realigning')
            self.enter('ALIGN', realign=True)
            return
        self.set_spray(True)

    # ---------------------------------------------------------------- VERIFY
    def enter_verify(self):
        self.set_spray(False)
        self.drive(0.0)
        self.sub['turned'] = 0.0
        self.sub['last_yaw'] = None

    def step_verify(self):
        # 360 deg thermal re-check in place. Rotates with cmd_vel instead of the Nav2 spin,
        # because spray particles may still be marked as obstacles in the local costmap.
        if self.detected:
            self.drive(0.0)
            self.retries += 1
            if self.retries > self.max_retries:
                self.nav.error('Fire still detected, giving up on this fire, resuming patrol')
                self.finish_fire()
                return
            self.nav.warn('Fire still detected, re-estimating and retrying')
            self.enter('FIRE_CONFIRMED')
            return
        if self.elapsed(self.state_time) > self.verify_timeout:
            self.drive(0.0)
            self.nav.warn('Verify scan timeout, resuming patrol')
            self.finish_fire()
            return
        pose = self.robot_pose()
        if pose is None:
            return
        if self.sub['last_yaw'] is not None:
            self.sub['turned'] += abs(wrap(pose[2] - self.sub['last_yaw']))
        self.sub['last_yaw'] = pose[2]
        if self.sub['turned'] >= 2.0 * math.pi:
            self.drive(0.0)
            self.nav.info('Fire extinguished, resuming patrol')
            self.finish_fire()
            return
        self.drive(self.scan_speed)

    def finish_fire(self):
        self.fire = None
        self.reported = False
        self.enter('PATROL')

    # ------------------------------------------------------------------ main
    def wait_sensor_offsets(self):
        """Read thermal camera and LiDAR positions in base_link (static TF from the robot
        description), so the mission does not depend on where the body mounts them.
        Both are assumed to face forward (yaw 0); the camera must be on the center line,
        since ALIGN turns the body heading onto the camera bearing."""
        while rclpy.ok():
            try:
                cam = self.tf_buffer.lookup_transform('base_link', 'thermal_link', Time())
                lid = self.tf_buffer.lookup_transform('base_link', 'lidar_link', Time())
            except Exception:
                rclpy.spin_until_future_complete(self.nav, Future(), timeout_sec=LOOP_PERIOD)
                continue
            cam, lid = cam.transform.translation, lid.transform.translation
            self.thermal_xy, self.lidar_xy = (cam.x, cam.y), (lid.x, lid.y)
            self.nav.info(f'Sensors in base_link: thermal ({cam.x:.3f}, {cam.y:.3f}), '
                          f'LiDAR ({lid.x:.3f}, {lid.y:.3f})')
            return

    def run(self):
        # waitUntilNav2Active() would re-publish a (0, 0) initial pose; AMCL already has one.
        self.nav._waitForNodeToActivate('amcl')
        self.nav._waitForNodeToActivate('bt_navigator')
        self.wait_sensor_offsets()
        self.started_pub.publish(Empty())
        self.nav.info(f'Patrol started: {len(self.waypoints)} waypoints')
        self.round_start = self.now()
        self.enter('PATROL')
        while rclpy.ok():
            # Process every pending callback for one loop period, then step the state machine
            rclpy.spin_until_future_complete(self.nav, Future(), timeout_sec=LOOP_PERIOD)
            self.step()


def main():
    rclpy.init()
    nav = BasicNavigator('mission_node')
    try:
        Mission(nav).run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
