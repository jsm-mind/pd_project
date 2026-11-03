"""Fixed voice announcements to the residents: key on /voice -> sentence.

Each line is played from a pre-made file voice/<key>.wav (tools/make_voice.py);
no speech synthesis at run time. Replace a wav with a recording of the same name
to change the voice.
"""

LINES = {
    'fire_detected': '화재가 발생했습니다.',
    'fire_alarm': '화재 경보가 울렸습니다.',
    'fire_confirmed': '화재를 확인했습니다.',
    'called_119': '119에 신고했습니다.',
    'notified_guardian': '보호자에게 연락했습니다.',
    'asked_check': '보호자에게 확인을 요청했습니다.',
    'evac_found': '화재가 발생했습니다. 저를 따라 대피하세요.',
    'evac_guide': '출구는 이쪽입니다. 저를 따라오세요.',
    'evac_done': '대피가 완료되었습니다. 건물 밖에서 기다려 주세요.',
    'suppression_start': '소화 장치를 작동합니다. 가까이 오지 마세요.',
    'suppression_done': '화재 진압을 완료했습니다.',
}
