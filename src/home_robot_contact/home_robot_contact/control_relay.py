"""Fire station relay screen: serves the page and pushes live robot data (SSE).

Open http://<jetson ip>:<port>/ from another device on the same network.
Only during EMERGENCY are the position, thermal image, map and log sent;
otherwise clients get {'standby': true} only.

Input : /robot_status  (std_msgs/String, latched) - JSON from home_robot_mode robot_status
        /thermal/image_full (sensor_msgs/Image, 224x120 Gazebo render, kelvin / 0.1) -
            shown on screen; detection still uses the 32x24 /thermal/image
        /guardian_message (std_msgs/String) - JSON {title, text, address} from contact
        TF map -> base_footprint
HTTP  : GET /        page (web/control_relay.html)
        GET /map     house map (pgm pixels, base64) - 403 outside EMERGENCY
        GET /events  text/event-stream: status, log, sms, pose (pose_rate), thermal (thermal_rate), hb
"""
import base64
import json
import math
import os
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

HEARTBEAT = 2.0     # [s] lets the page show the link as up while on standby
GZ_RESOLUTION = 0.1  # kelvin per count of the Gazebo thermal image (URDF <resolution>)


def load_map(yaml_path):
    with open(yaml_path) as f:
        info = yaml.safe_load(f)
    with open(os.path.join(os.path.dirname(yaml_path), info['image']), 'rb') as f:
        raw = f.read()
    # binary pgm: 'P5' width height maxval, each followed by one whitespace
    fields, pos = [], 0
    while len(fields) < 4:
        while raw[pos:pos + 1].isspace():
            pos += 1
        end = pos
        while not raw[end:end + 1].isspace():
            end += 1
        fields.append(raw[pos:end])
        pos = end
    width, height = int(fields[1]), int(fields[2])
    pixels = raw[pos + 1:pos + 1 + width * height]
    return json.dumps({'width': width, 'height': height, 'resolution': info['resolution'],
                       'origin': info['origin'][:2],
                       'data': base64.b64encode(pixels).decode()}).encode()


class ControlRelay(Node):
    def __init__(self):
        super().__init__('control_relay')
        nav_maps = os.path.join(get_package_share_directory('home_robot_nav'), 'maps')
        port = self.declare_parameter('port', 8080).value
        map_yaml = self.declare_parameter('map', os.path.join(nav_maps, 'house.yaml')).value
        pose_rate = self.declare_parameter('pose_rate', 2.0).value
        thermal_rate = self.declare_parameter('thermal_rate', 2.0).value

        page = os.path.join(get_package_share_directory('home_robot_contact'),
                            'web', 'control_relay.html')
        with open(page, 'rb') as f:
            self.page = f.read()
        self.map = load_map(map_yaml)

        self.tf = Buffer()
        TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_status', self.on_status, latched)
        self.create_subscription(Image, 'thermal/image_full', self.on_thermal, 10)
        self.create_subscription(String, 'guardian_message', self.on_guardian, 10)
        self.create_timer(1.0 / pose_rate, self.send_pose)
        self.create_timer(1.0 / thermal_rate, self.send_thermal)
        self.create_timer(HEARTBEAT, lambda: self.broadcast('hb', {}))

        self.lock = threading.RLock()
        self.streams = []         # one queue per open /events stream
        self.emergency = False
        self.status = {'standby': True}
        self.log = []             # events of the current emergency
        self.sms = []             # guardian messages of the current emergency
        self.start = 0.0
        self.thermal = None       # latest image, converted only when sent

        server = ThreadingHTTPServer(('0.0.0.0', port), self.handler())
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.get_logger().info(f'Relay screen on http://<this host>:{port}/')

    # ---------- inputs ----------
    def on_status(self, msg):
        status = json.loads(msg.data)
        event = status.pop('event')
        emergency = status['mode'] == 'EMERGENCY'
        if emergency and not self.emergency:
            self.start = time.time()
            with self.lock:
                self.log = []
                self.sms = []
        self.emergency = emergency
        self.status = status if emergency else {'standby': True}
        self.broadcast('status', self.snapshot())
        if emergency and event:
            now = time.time()
            entry = {'t': now - self.start, 'clock': time.strftime('%H:%M:%S', time.localtime(now)),
                     'level': event[0], 'text': event[1]}
            with self.lock:     # no new client between the two, or it gets this entry twice
                self.log.append(entry)
                self.broadcast('log', entry)

    def on_guardian(self, msg):
        if not self.emergency:
            return
        now = time.time()
        entry = dict(json.loads(msg.data), t=now - self.start,
                     clock=time.strftime('%H:%M', time.localtime(now)))
        with self.lock:
            self.sms.append(entry)
            self.broadcast('sms', entry)

    def snapshot(self):
        # elapsed lets a page opened mid-emergency show the same T+ as the log
        if not self.emergency:
            return self.status
        return dict(self.status, elapsed=round(time.time() - self.start, 1))

    def on_thermal(self, msg):
        self.thermal = msg

    # ---------- periodic output (EMERGENCY only) ----------
    def send_pose(self):
        if not self.emergency:
            return
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time()).transform
        except Exception:
            return
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.broadcast('pose', {'x': round(t.translation.x, 3), 'y': round(t.translation.y, 3),
                                'yaw': round(yaw, 3)})

    def send_thermal(self):
        msg = self.thermal
        if not self.emergency or msg is None:
            return
        if msg.encoding == '32FC1':
            temp = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        else:   # 16-bit Gazebo thermal render
            raw = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)
            temp = raw.astype(np.float32) * GZ_RESOLUTION - 273.15
        lo, hi = float(temp.min()), float(temp.max())
        level = np.round((temp - lo) / max(hi - lo, 1e-3) * 255).astype(np.uint8)
        self.broadcast('thermal', {'w': msg.width, 'h': msg.height, 'min': round(lo, 1),
                                   'max': round(hi, 1),
                                   'data': base64.b64encode(level.tobytes()).decode()})

    def broadcast(self, kind, data):
        chunk = f'event: {kind}\ndata: {json.dumps(data, ensure_ascii=False)}\n\n'.encode()
        with self.lock:
            for q in self.streams:
                q.put(chunk)

    # ---------- HTTP ----------
    def handler(self):
        relay = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def reply(self, body, kind):
                self.send_response(200)
                self.send_header('Content-Type', kind)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == '/':
                    self.reply(relay.page, 'text/html; charset=utf-8')
                elif self.path == '/map':
                    if relay.emergency:
                        self.reply(relay.map, 'application/json')
                    else:
                        self.send_error(403)
                elif self.path == '/events':
                    self.stream()
                else:
                    self.send_error(404)

            def stream(self):
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                q = queue.Queue()
                with relay.lock:
                    # snapshot first, then live updates in order
                    q.put(f'event: status\ndata: {json.dumps(relay.snapshot(), ensure_ascii=False)}\n\n'.encode())
                    for entry in (relay.log if relay.emergency else []):
                        q.put(f'event: log\ndata: {json.dumps(entry, ensure_ascii=False)}\n\n'.encode())
                    for entry in (relay.sms if relay.emergency else []):
                        q.put(f'event: sms\ndata: {json.dumps(entry, ensure_ascii=False)}\n\n'.encode())
                    relay.streams.append(q)
                try:
                    while True:
                        self.wfile.write(q.get())
                        self.wfile.flush()
                except OSError:
                    pass
                finally:
                    with relay.lock:
                        relay.streams.remove(q)

        return Handler


def main():
    rclpy.init()
    rclpy.spin(ControlRelay())
