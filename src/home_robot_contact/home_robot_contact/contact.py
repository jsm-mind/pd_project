"""119 phone call and guardian/resident contact.

Input : /contact_request (std_msgs/String) - request names in MESSAGES or 'call_119'
        /fire_location   (PoseStamped, transient_local) - shown in messages
        /alarm_location  (PoseStamped, transient_local) - shown when no fire location yet
        /fire_seen       (std_msgs/Float32) - first fire sighting: relay-screen SMS only
Output: Twilio call to call_number (or a log line when call_enabled is false),
        boxed messages on the terminal
        /guardian_message (std_msgs/String) - JSON {title, text, address} of each
        terminal message, shown as an SMS on the relay page
"""
import base64
import json
import threading
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32, String

TWILIO_URL = 'https://api.twilio.com/2010-04-01/Accounts/{sid}/Calls.json'
# Trial accounts reject the inline 'Twiml' parameter, so the TwiML is served
# back to Twilio by the public echo twimlet and passed as 'Url'.
ECHO_URL = 'https://twimlets.com/echo?'

# Terminal messages to the guardian / resident
MESSAGES = {
    'notify_fire': '화재가 발생했습니다. 119에 신고했습니다. 즉시 대피하세요.',
    'ask_check': '화재 경보가 울렸습니다. 현장 확인을 부탁드립니다.',
    'notify_confirmed': '화재가 확인되었습니다. 119에 신고했습니다. 즉시 대피하세요.',
    'notify_done': '화재 진압을 완료했습니다.',
}
TITLES = {
    'notify_fire': '화재 발생', 'ask_check': '확인 요청',
    'notify_confirmed': '화재 확인', 'notify_done': '진압 완료',
}


class Contact(Node):
    def __init__(self):
        super().__init__('contact')
        self.address = self.declare_parameter('address', '').value
        self.number = self.declare_parameter('call_number', '').value
        self.call_enabled = self.declare_parameter('call_enabled', False).value
        self.sid = self.declare_parameter('twilio_sid', '').value
        self.token = self.declare_parameter('twilio_token', '').value
        self.caller = self.declare_parameter('twilio_from', '').value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.voice = self.create_publisher(String, 'voice', 10)
        self.guardian = self.create_publisher(String, 'guardian_message', 10)
        self.create_subscription(String, 'contact_request', self.on_request, 10)
        self.create_subscription(PoseStamped, 'fire_location', self.on_location, latched)
        self.create_subscription(PoseStamped, 'alarm_location', self.on_alarm, latched)
        self.create_subscription(Float32, 'fire_seen', self.on_fire_seen, 10)
        self.location = None
        self.alarm = None

    # ---------- inputs ----------
    def on_location(self, msg):
        self.location = (msg.pose.position.x, msg.pose.position.y)

    def on_alarm(self, msg):
        self.alarm = (msg.pose.position.x, msg.pose.position.y)

    def on_fire_seen(self, msg):
        # screen-only SMS; the terminal contact flow stays as it is
        self.guardian.publish(String(data=json.dumps(
            {'title': '화재 발견', 'text': f'HOLMES가 열화상으로 화재를 발견했습니다. ({msg.data:.0f}℃)',
             'address': self.address}, ensure_ascii=False)))

    def on_request(self, msg):
        self.dispatch(msg.data)

    def dispatch(self, request):
        if request == 'call_119':
            self.call_119()
            self.say('called_119')
        elif request in MESSAGES:
            self.notify(MESSAGES[request])
            self.guardian.publish(String(data=json.dumps(
                {'title': TITLES[request], 'text': MESSAGES[request], 'address': self.address},
                ensure_ascii=False)))
            # only the fact of the contact is announced, not its content
            self.say('asked_check' if request == 'ask_check' else 'notified_guardian')
        else:
            self.get_logger().warn(f'Unknown contact request "{request}"')

    # ---------- outputs ----------
    def notify(self, text):
        if self.location is not None:
            where = f'화재 위치(맵 좌표): ({self.location[0]:.1f}, {self.location[1]:.1f})'
        elif self.alarm is not None:
            where = f'경보 위치(맵 좌표): ({self.alarm[0]:.1f}, {self.alarm[1]:.1f})'
        else:
            where = '화재 위치(맵 좌표): -'
        lines = [
            '[보호자/거주자 연락]',
            f'시각: {datetime.now():%Y-%m-%d %H:%M:%S}',
            f'주소: {self.address}',
            where,
            f'내용: {text}',
        ]
        bar = '=' * 60
        print('\n'.join([bar, *lines, bar]), flush=True)

    def say(self, key):
        self.voice.publish(String(data=key))

    def call_119(self):
        speech = f'단독주택 화재 신고입니다. 주소는 {self.address} 입니다. 거주자가 있을 수 있습니다.'
        if not self.call_enabled:
            self.get_logger().info(f'[call disabled] would call {self.number}: "{speech}"')
            return
        self.get_logger().info(f'Calling {self.number} (119 stand-in)')
        # HTTP request off the executor thread
        threading.Thread(target=self.twilio_call, args=(speech,), daemon=True).start()

    def twilio_call(self, speech):
        twiml = (f'<Response><Say language="ko-KR" voice="Polly.Seoyeon" loop="2">'
                 f'{speech}</Say></Response>')
        body = urllib.parse.urlencode(
            {'To': self.number, 'From': self.caller,
             'Url': ECHO_URL + urllib.parse.urlencode({'Twiml': twiml})}).encode()
        req = urllib.request.Request(TWILIO_URL.format(sid=self.sid), data=body)
        auth = base64.b64encode(f'{self.sid}:{self.token}'.encode()).decode()
        req.add_header('Authorization', f'Basic {auth}')
        try:
            with urllib.request.urlopen(req, timeout=10) as res:
                self.get_logger().info(f'Call placed (HTTP {res.status})')
        except urllib.error.HTTPError as e:
            self.get_logger().error(f'Call failed: {e} {e.read().decode(errors="replace")}')
        except Exception as e:  # noqa: BLE001 - report any failure, the scenario goes on
            self.get_logger().error(f'Call failed: {e}')


def main():
    rclpy.init()
    rclpy.spin(Contact())
