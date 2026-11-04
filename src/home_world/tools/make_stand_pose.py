#!/usr/bin/env python3
# usage: make_stand_pose.py models/male_visitor/meshes/MaleVisitorWalk.dae models/male_visitor/meshes/MaleVisitorStand.dae
"""Build a standing-pose DAE from the walk DAE: every keyframe of every channel
is replaced by one neutral matrix. Torso and legs use the skeleton rest pose
(upright, straight legs); arms use the mean swing of the walk with the
straightest elbow (the rest pose has T-pose arms)."""
import sys, re, numpy as np
from scipy.spatial.transform import Rotation as Rot
src, out = sys.argv[1], sys.argv[2]
t = open(src).read()
vs = t[t.index('<library_visual_scenes'):]
rest = {m.group(1): np.array(m.group(2).split(), float).reshape(4, 4) for m in re.finditer(
    r'<node id="Armature_([^"]+)"[^>]*type="JOINT">\s*<matrix sid="transform">([^<]+)<', vs)}
samp = {m.group(1): m.group(2) for m in re.finditer(r'<sampler id="([^"]+)">(.*?)</sampler>', t, re.S)}
for s, tg in re.findall(r'<channel source="#([^"]+)" target="([^"]+)"', t):
    joint = tg.split('/')[0].replace('Armature_', '')
    out_id = re.search(r'semantic="OUTPUT" source="#([^"]+)"', samp[s]).group(1) + '-array'
    m = re.search(r'(<float_array id="%s" count="(\d+)">)([^<]*)(<)' % re.escape(out_id), t)
    a = np.array(m.group(3).split(), float).reshape(-1, 4, 4)
    if 'Arm' not in joint:
        n = rest[joint]
    elif joint.startswith('LowerArm'):
        bend = [np.degrees(np.arccos(np.clip(f[1, 1] / np.linalg.norm(f[:3, 1]), -1, 1))) for f in a]
        n = a[int(np.argmin(bend))].copy()
    else:
        n = np.eye(4)
        n[:3, :3] = Rot.from_matrix(a[:, :3, :3]).mean().as_matrix()
        n[:3, 3] = a[:, :3, 3].mean(0)
    vals = ' '.join('%.6f' % v for v in np.tile(n.reshape(-1), len(a)))
    t = t[:m.start(3)] + vals + t[m.end(3):]
open(out, 'w').write(t)
print('ok')
