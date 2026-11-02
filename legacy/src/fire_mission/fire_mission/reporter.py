"""Single place for all reports (currently printed to the terminal).

The registered address is read from a local YAML file kept out of the repository
(see config/report.example.yaml).
"""
import datetime
import os

import yaml

# kind -> (label, recipient)
KINDS = {
    'FIRE_CONFIRMED': ('화재 확인', '119 신고'),
    'ALARM_NO_FIRE': ('경보 지속, 화재 미확인', '119 신고'),
    'EVACUATING': ('진압 불능, 대피 유도 중', '119 신고'),
    'OWNER_NOTICE': ('경보 해제, 화재 미발견 (오경보 의심)', '집주인 알림'),
}
NO_ADDRESS = '(주소 미등록)'


class Reporter:

    def __init__(self, logger, config_path):
        self.address = NO_ADDRESS
        path = os.path.expanduser(config_path)
        try:
            with open(path) as f:
                self.address = (yaml.safe_load(f) or {}).get('address') or NO_ADDRESS
        except OSError:
            logger.warn(f'Report config {path} not found, address not registered '
                        '(copy fire_mission/config/report.example.yaml there)')

    def report(self, kind, where=None):
        """Print one report. where: estimated fire location text, if known."""
        label, recipient = KINDS[kind]
        stamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        line = f'[{stamp}] {recipient} — {label} | 주소: {self.address}'
        if where:
            line += f' | 추정 위치: {where}'
        print(f'\n{line}\n', flush=True)
