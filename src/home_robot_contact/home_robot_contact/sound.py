"""Emergency sounds (non-blocking; each sound is its own player process).

Input : /mode              (std_msgs/String, latched) - siren once on entering EMERGENCY
                             (sox trims to siren_end s with a short fade-out, original untouched)
        /fire_alarm_sound  (std_msgs/Empty) - smoke alarm sound starts, looped
        /fire_extinguished (std_msgs/Empty) - smoke alarm sound stops
Volumes are paplay/pacat volumes (65536 = 100 %, the voice lines' level).
"""
import os
import subprocess
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String


class Sound(Node):
    def __init__(self):
        super().__init__('sound')
        d = os.path.join(get_package_share_directory('home_robot_contact'), 'sound')
        self.siren_wav = os.path.join(d, 'siren.wav')
        self.alarm_wav = os.path.join(d, 'smoke_detector_alarm.wav')
        self.siren_volume = self.declare_parameter('siren_volume', 0.5).value
        self.alarm_volume = self.declare_parameter('alarm_volume', 0.3).value
        self.siren_end = self.declare_parameter('siren_end', 8.0).value     # [s]
        self.siren_fade = self.declare_parameter('siren_fade', 0.7).value   # [s]
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'mode', self.on_mode, latched)
        self.create_subscription(Empty, 'fire_alarm_sound', self.on_alarm, 10)
        self.create_subscription(Empty, 'fire_extinguished', self.on_extinguished, 10)
        self.mode = None
        self.alarm_on = threading.Event()
        self.alarm_proc = None

    # ---------- siren ----------
    def on_mode(self, msg):
        prev, self.mode = self.mode, msg.data
        if self.mode == 'EMERGENCY' and prev is not None and prev != 'EMERGENCY':
            self.get_logger().info('Siren')
            threading.Thread(target=self.play_siren, daemon=True).start()

    def play_siren(self):
        try:
            sox = subprocess.Popen(
                ['sox', '-q', self.siren_wav, '-t', 'raw', '-e', 'signed-integer', '-b', '16',
                 '-c', '2', '-r', '44100', '-', 'trim', '0', str(self.siren_end),
                 'fade', 't', '0', str(self.siren_end), str(self.siren_fade)],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            subprocess.run(['pacat', '--rate=44100', '--channels=2', '--format=s16le',
                            f'--volume={self.vol(self.siren_volume)}'],
                           stdin=sox.stdout, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            sox.wait()
        except OSError:
            pass

    # ---------- smoke alarm ----------
    def on_alarm(self, _):
        if self.alarm_on.is_set():
            return
        self.get_logger().info('Smoke alarm sound on')
        self.alarm_on.set()
        threading.Thread(target=self.loop_alarm, daemon=True).start()

    def on_extinguished(self, _):
        if not self.alarm_on.is_set():
            return
        self.get_logger().info('Smoke alarm sound off (fire extinguished)')
        self.alarm_on.clear()
        if self.alarm_proc is not None:
            self.alarm_proc.terminate()

    def loop_alarm(self):
        while self.alarm_on.is_set():
            try:
                self.alarm_proc = subprocess.Popen(
                    ['paplay', f'--volume={self.vol(self.alarm_volume)}', self.alarm_wav],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                if not self.alarm_on.is_set():   # stopped while starting
                    self.alarm_proc.terminate()
                self.alarm_proc.wait()
            except OSError:
                return

    @staticmethod
    def vol(fraction):
        return int(65536 * fraction)


def main():
    rclpy.init()
    rclpy.spin(Sound())
