#!/usr/bin/env python3
"""Build the pre-provided occupancy map from the house world SDF.

Rasterizes static box collisions that overlap the robot's height band.
Only <model> elements are used; included models (charger, resident) and
the spawned fire are excluded, plus EXCLUDE below for safety.

usage: make_map.py <world.sdf> <out_dir>
"""
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

RES = 0.05
X_MIN, X_MAX, Y_MIN, Y_MAX = -1.0, 12.0, -1.0, 9.5
Z_LOW, Z_HIGH = 0.12, 0.47      # world z band: floor top 0.1 + robot height
EXCLUDE = {'ground_plane', 'charger', 'resident', 'fire'}


def pose(el):
    txt = el.findtext('pose') if el is not None else None
    return [float(v) for v in txt.split()] if txt else [0.0] * 6


def main(world_file, out_dir):
    world = ET.parse(world_file).getroot().find('world')
    w = int(round((X_MAX - X_MIN) / RES))
    h = int(round((Y_MAX - Y_MIN) / RES))
    occ = np.zeros((h, w), dtype=bool)

    for model in world.findall('model'):
        if model.get('name') in EXCLUDE:
            continue
        mx, my, mz, _, _, myaw = pose(model)
        for link in model.findall('link'):
            lx, ly, lz = pose(link)[:3]
            for col in link.findall('collision'):
                box = col.find('geometry/box')
                if box is None:
                    continue
                sx, sy, sz = (float(v) for v in box.findtext('size').split())
                cx, cy, cz = pose(col)[:3]
                # all house collisions are axis-aligned; model yaw is 0 or pi
                px, py = lx + cx, ly + cy
                c, s = np.cos(myaw), np.sin(myaw)
                wx, wy, wz = mx + c * px - s * py, my + s * px + c * py, mz + lz + cz
                if wz + sz / 2 < Z_LOW or wz - sz / 2 > Z_HIGH:
                    continue
                if abs(s) > 0.5:
                    sx, sy = sy, sx
                # mark cells whose center lies inside the box (no outward growth),
                # but keep at least one cell so thin walls are not lost
                i0 = int(round((wx - sx / 2 - X_MIN) / RES))
                i1 = max(int(round((wx + sx / 2 - X_MIN) / RES)), i0 + 1)
                j0 = int(round((wy - sy / 2 - Y_MIN) / RES))
                j1 = max(int(round((wy + sy / 2 - Y_MIN) / RES)), j0 + 1)
                occ[max(j0, 0):max(j1, 0), max(i0, 0):max(i1, 0)] = True

    img = np.where(occ, 0, 254).astype(np.uint8)[::-1]  # row 0 = top (max y)
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'house.pgm'), 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(img.tobytes())
    with open(os.path.join(out_dir, 'house.yaml'), 'w') as f:
        f.write('image: house.pgm\nmode: trinary\n'
                f'resolution: {RES}\norigin: [{X_MIN}, {Y_MIN}, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'map {w}x{h} written to {out_dir}')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
