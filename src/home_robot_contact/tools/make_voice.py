"""Make voice/<key>.wav for every line in voice_lines.LINES (run once, needs internet).

Usage: python3 src/home_robot_contact/tools/make_voice.py   (then colcon build)
Needs: pip install gTTS, ffmpeg
"""
import os
import subprocess
import sys
import tempfile

from gtts import gTTS

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PKG, 'home_robot_contact'))
from voice_lines import LINES  # noqa: E402

OUT = os.path.join(PKG, 'voice')

os.makedirs(OUT, exist_ok=True)
with tempfile.TemporaryDirectory() as tmp:
    for key, text in LINES.items():
        mp3 = os.path.join(tmp, key + '.mp3')
        gTTS(text, lang='ko').save(mp3)
        # 22.05 kHz mono 16-bit wav: plain paplay/aplay playback
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', mp3, '-ar', '22050', '-ac', '1',
                        os.path.join(OUT, key + '.wav')], check=True)
        print(f'{key}.wav  {text}')
