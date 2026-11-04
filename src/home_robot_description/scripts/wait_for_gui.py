#!/usr/bin/env python3
"""Exit once the Gazebo GUI has built its scene (or after a timeout without a GUI).

An entity created while the GUI is still loading the initial world state is
missed by the GUI and never drawn, so the robot is spawned only after this exits.
The GUI advertises /gui/camera/pose once its scene exists.
"""
import os
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')  # gz python msgs

import sys  # noqa: E402
import time  # noqa: E402

from gz.transport13 import Node  # noqa: E402

TOPIC = '/gui/camera/pose'
TIMEOUT = 10.0   # [s] headless runs have no GUI; Nav2 starts at 15 s


def main():
    node = Node()
    start = time.time()
    while time.time() - start < TIMEOUT:
        if TOPIC in node.topic_list():
            print('Gazebo GUI ready, spawning the robot')
            return
        time.sleep(0.5)
    print(f'no Gazebo GUI after {TIMEOUT:.0f} s, spawning the robot')


if __name__ == '__main__':
    sys.exit(main())
