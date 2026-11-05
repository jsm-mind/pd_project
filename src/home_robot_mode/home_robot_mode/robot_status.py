"""Single source of the robot's current mode/step, for the status ring and the
fire station relay screen.

Input : /mode                (std_msgs/String, latched) - PATROL / CHARGING / EMERGENCY
        /mode_event          (std_msgs/String) - 'go_charge', 'fire_detected', 'fire_alarm'
        /contact_request     (std_msgs/String) - 119 / guardian contact requests
        /person_location     (geometry_msgs/PointStamped) - resident seen
        /projector           (std_msgs/Bool)   - on while guiding the evacuation
        /evacuation_status   (std_msgs/String) - 'done' / 'not_found' / 'failed'
        /fire_location       (geometry_msgs/PoseStamped, latched) - fire seen
        /suppression_status  (std_msgs/String) - 'releasing' / 'done'
        /fire_seen           (std_msgs/Float32) - first fire in view this emergency
Output: /robot_status (std_msgs/String, latched) - JSON, on every change:
        mode, stage, alert, cause, fire, fire_seen, resident, called_119,
        event: [level, text] that caused this update, or null
stage : patrol / returning / charging /
        report -> searching -> guiding -> to_fire -> suppressing -> done (EMERGENCY)
alert : true for ALERT_TIME s after the emergency starts or a 119/contact request
"""
import json
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32, String

ALERT_TIME = 3.0
CAUSES = {
    'fire_detected': ('crit', '열화상으로 화재 감지 (80℃ 이상) · 긴급모드'),
    'fire_alarm': ('crit', '화재 경보 수신 · 긴급모드'),
}
CONTACTS = {
    'call_119': ('crit', '119 지정 번호로 신고 전화 발신'),
    'notify_fire': ('info', '보호자·거주자에게 화재 발생 연락'),
    'ask_check': ('info', '보호자·거주자에게 현장 확인 요청'),
    'notify_confirmed': ('info', '보호자·거주자에게 화재 확인 연락'),
    'notify_done': ('ok', '보호자·거주자에게 진압 완료 연락'),
}
EVACUATION = {
    'done': ('evacuated', 'ok', '거주자 대피 완료 · 화재 위치로 이동'),
    'not_found': ('not_found', 'warn', '거주자를 찾지 못함 · 화재 위치로 이동'),
    'failed': ('failed', 'warn', '대피 목적지 도착 실패 · 화재 위치로 이동'),
}


class RobotStatus(Node):
    def __init__(self):
        super().__init__('robot_status')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, 'robot_status', latched)
        self.create_subscription(String, 'mode', self.on_mode, latched)
        self.create_subscription(String, 'mode_event', self.on_event, 10)
        self.create_subscription(String, 'contact_request', self.on_contact, 10)
        self.create_subscription(PointStamped, 'person_location', self.on_person, 10)
        self.create_subscription(Bool, 'projector', self.on_projector, 10)
        self.create_subscription(String, 'evacuation_status', self.on_evacuation, 10)
        self.create_subscription(PoseStamped, 'fire_location', self.on_fire, latched)
        self.create_subscription(String, 'suppression_status', self.on_suppression, 10)
        self.create_subscription(Float32, 'fire_seen', self.on_fire_seen, 10)

        self.mode = 'CHARGING'
        self.stage = 'charging'
        self.cause = None         # 'fire_detected' | 'fire_alarm'; may arrive before /mode
        self.alert_until = 0.0
        self.reset_emergency()
        self.publish()
        self.create_timer(0.2, self.check_alert)

    def reset_emergency(self):
        self.fire = None          # 'alarm' (not seen yet) | 'burning' | 'suppressing' | 'out'
        self.resident = None      # 'found' | 'guiding' | 'evacuated' | 'not_found' | 'failed'
        self.called_119 = False
        self.fire_seen = False

    # ---------- inputs ----------
    def on_mode(self, msg):
        if msg.data == self.mode:
            return
        prev, self.mode = self.mode, msg.data
        if self.mode == 'EMERGENCY':
            self.stage = 'report'
            self.alert_until = time.monotonic() + ALERT_TIME
            self.reset_emergency()
            if self.cause:
                self.start_cause()
            else:
                self.publish(('crit', '긴급모드 진입'))
            return
        if prev == 'EMERGENCY':
            self.cause = None
        self.stage = 'patrol' if self.mode == 'PATROL' else 'charging'
        self.publish(('info', '순찰모드' if self.mode == 'PATROL' else '충전모드'))

    def on_event(self, msg):
        if msg.data == 'go_charge' and self.mode == 'PATROL':
            self.stage = 'returning'
            self.publish(('info', '충전기로 복귀'))
        elif msg.data in CAUSES and self.cause is None:
            self.cause = msg.data
            if self.mode == 'EMERGENCY':
                self.start_cause()

    def start_cause(self):
        self.fire = 'burning' if self.cause == 'fire_detected' else 'alarm'
        self.publish(CAUSES[self.cause])

    def on_contact(self, msg):
        if self.mode != 'EMERGENCY' or msg.data not in CONTACTS:
            return
        if msg.data == 'call_119':
            self.called_119 = True
        if msg.data == 'notify_done':
            self.stage = 'done'
        else:
            self.alert_until = time.monotonic() + ALERT_TIME
        self.publish(CONTACTS[msg.data])

    def on_person(self, msg):
        if self.mode == 'EMERGENCY' and self.stage == 'searching' and self.resident is None:
            self.resident = 'found'
            self.publish(('warn', '열화상으로 거주자 발견'))

    def on_projector(self, msg):
        if msg.data and self.mode == 'EMERGENCY':
            self.stage = 'guiding'
            self.resident = 'guiding'
            self.publish(('info', '빔프로젝터 화살표 · 음성으로 대피유도 시작'))

    def on_evacuation(self, msg):
        if self.mode != 'EMERGENCY' or msg.data not in EVACUATION:
            return
        self.resident, level, text = EVACUATION[msg.data]
        self.stage = 'to_fire'
        self.publish((level, text))

    def on_fire(self, msg):
        if self.mode == 'EMERGENCY' and self.fire == 'alarm':
            self.fire = 'burning'
            self.publish(('crit', '열화상으로 화재 확인'))

    def on_fire_seen(self, msg):
        if self.mode != 'EMERGENCY' or self.fire_seen:
            return
        self.fire_seen = True
        if self.fire == 'alarm':
            self.fire = 'burning'
        self.publish(('crit', f'열화상으로 화재 발견 ({msg.data:.0f}℃)'))

    def on_suppression(self, msg):
        if self.mode != 'EMERGENCY':
            return
        if msg.data == 'releasing':
            self.stage = 'suppressing'
            self.fire = 'suppressing'
            self.publish(('crit', '고체 에어로졸 소화장치 방출 · 진압 시작'))
        elif msg.data == 'done':
            self.fire = 'out'
            self.publish(('ok', '화재 진압 완료'))

    def check_alert(self):
        alert = time.monotonic() < self.alert_until
        if alert == self.alert:
            return
        if not alert and self.stage == 'report':
            self.stage = 'searching'
        self.publish()

    # ---------- output ----------
    def publish(self, event=None):
        self.alert = time.monotonic() < self.alert_until
        status = {
            'mode': self.mode, 'stage': self.stage, 'alert': self.alert,
            'cause': self.cause if self.mode == 'EMERGENCY' else None,
            'fire': self.fire, 'fire_seen': self.fire_seen,
            'resident': self.resident, 'called_119': self.called_119,
            'event': event,
        }
        self.pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        if event:
            self.get_logger().info(f'[{self.stage}] {event[1]}')


def main():
    rclpy.init()
    rclpy.spin(RobotStatus())
