#!/usr/bin/env python3
"""Derive public/mujoco/g1_upper.xml from MuJoCo Menagerie's unitree_g1/g1_with_hands.xml.

Usage: python3 scripts/build-g1-mjcf.py /path/to/mujoco_menagerie/unitree_g1

The Menagerie commit of the checkout used is recorded in the output file's first line.

Meshes are split between physics and rendering. MuJoCo only ever collides with a mesh's convex hull, so the
collision geoms reference precomputed hulls (public/models/meshes/hulls/<name>.STL, 27 files, ~70k faces):
identical collision shapes, but the physics worker no longer compiles 630k triangles (798 MB of WASM memory
and 1.4 s on load became 131 MB and 0.4 s). The visual meshes are not in the MJCF at all: the full-resolution
STL files are copied from Menagerie's assets folder into public/models/meshes (the operator looks at the
hands all day from 30 cm away) and public/models/meshes/visual.json lists which file hangs off which body at
what offset, for the renderer to load itself. Requires Python >= 3.11 (see requirements-tools.txt).

Changes vs. the Menagerie model (BSD-3, see public/mujoco/LICENSE-g1):
  - fixed base: pelvis freejoint removed (upper-body manipulation, like Isaac's FixedBaseUpperBodyIK task)
  - legs kept as static visuals (joints, collision geoms and actuators removed)
  - gravity compensation on arm + hand bodies (the real arm controller adds a gravity feed-forward)
  - actuator PD gains set to what Unitree's own teleop stack (xr_teleoperate) sends to the real robot:
    shoulder/elbow kp=80 kd=3, wrist kp=40 kd=1.5, waist kp=300 kd=3, Dex3 fingers kp=1.5 kd=0.2
  - Dex3 finger joints get their own friction/armature: Menagerie's body-wide frictionloss=0.3 Nm exceeds the
    finger motors' kp*error for small commands (a 0.17 rad command did not move the finger at all), so the hand
    joints use frictionloss=0.01, armature=0.0005
  - sites: eye (mid360 origin, used for VR calibration), head_cam (d435), {left,right}_palm (IK target),
    {left,right}_grip (center of the grasp, used by task checks)
  - meshdir points at the shared mesh folder (public/models/meshes)
"""
import re
import json
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

LEG = re.compile(r'hip|knee|ankle')
ARM_ROOT = re.compile(r'(left|right)_shoulder_pitch_link')

if len(sys.argv) != 2 or not (Path(sys.argv[1]) / 'g1_with_hands.xml').exists():
    raise SystemExit(__doc__)
src_dir = Path(sys.argv[1])
repo = Path(__file__).resolve().parent.parent
try:
    import subprocess
    menagerie_commit = subprocess.check_output(['git', '-C', str(src_dir), 'rev-parse', 'HEAD'], text=True).strip()
except Exception:  # not a git checkout
    menagerie_commit = 'unknown'

out = repo / 'public' / 'mujoco' / 'g1_upper.xml'

tree = ET.parse(src_dir / 'g1_with_hands.xml')
root = tree.getroot()
root.set('model', 'g1_29dof_with_hand_fixed_base')
root.find('compiler').set('meshdir', 'meshes')

for kf in root.findall('keyframe'):
    root.remove(kf)

bodies = {b.get('name'): b for b in root.iter('body')}

# Fixed base
pelvis = bodies['pelvis']
for fj in pelvis.findall('freejoint'):
    pelvis.remove(fj)

# Legs: static visuals only
for name, body in bodies.items():
    if not LEG.search(name):
        continue
    for el in list(body):
        if el.tag in ('joint', 'site') or (el.tag == 'geom' and el.get('class') in ('collision', 'foot')):
            body.remove(el)

leg_joint = re.compile(r'(hip|knee|ankle).*_joint')
for sec in root.findall('actuator') + root.findall('contact'):
    for el in list(sec):
        if any(leg_joint.search(v) for v in el.attrib.values()):
            sec.remove(el)
# The IMU sensors serve the locomotion controller; a fixed-base teleop sim never reads them and they cost ~2% of
# step time
for sec in root.findall('sensor'):
    root.remove(sec)

# Gravity compensation on both arm subtrees
for name, body in bodies.items():
    if ARM_ROOT.fullmatch(name):
        for b in body.iter('body'):
            b.set('gravcomp', '1')


# Joint-level PD gains from unitreerobotics/xr_teleoperate (robot_arm.py, robot_hand_unitree.py); MuJoCo's
# position actuator is tau = kp (q_des - q) - kv dq, i.e. the same law with dq_des = 0.
GAINS = [
    (re.compile(r'_hand_'), 1.5, 0.2),
    (re.compile(r'_wrist_'), 40, 1.5),
    (re.compile(r'shoulder|elbow'), 80, 3),
    (re.compile(r'waist'), 300, 3),
]
for pos in root.find('default').iter('position'):
    for k in ('kp', 'dampratio'):
        pos.attrib.pop(k, None)
for j in root.iter('joint'):
    if '_hand_' in j.get('name', ''):
        j.set('frictionloss', '0.01')
        j.set('armature', '0.0005')
for act in root.find('actuator'):
    for pat, kp, kv in GAINS:
        if pat.search(act.get('name', '')):
            act.set('kp', str(kp))
            act.set('kv', str(kv))
            break
    else:
        raise SystemExit(f"no gains for actuator {act.get('name')}")


def site(parent, name, pos, **extra):
    el = ET.SubElement(parent, 'site', {'name': name, 'pos': pos, 'size': '0.005', 'group': '5', **extra})
    return el


torso = bodies['torso_link']
# Origins of mid360_joint / d435_joint relative to torso_link in Unitree's g1 URDF
site(torso, 'eye', '0.0002835 0.00003 0.40618')
site(torso, 'head_cam', '0.0576235 0.01753 0.41987', euler='0 0.8307767 0')

# Palm origin = URDF {side}_hand_palm_joint; grip = between thumb and fingers (fingers curl toward -y on the
# left hand, +y on the right)
site(bodies['left_wrist_yaw_link'], 'left_palm', '0.0415 0.003 0')
site(bodies['right_wrist_yaw_link'], 'right_palm', '0.0415 -0.003 0')
site(bodies['left_wrist_yaw_link'], 'left_grip', '0.115 -0.035 0')
site(bodies['right_wrist_yaw_link'], 'right_grip', '0.115 0.035 0')

# Mesh split (see the docstring): visual mesh geoms leave the MJCF for visual.json, collision mesh geoms get
# precomputed convex hulls
mesh_file = {}
for m in root.iter('mesh'):
    mesh_file[m.get('name') or m.get('file').rsplit('.', 1)[0]] = m.get('file')
visual = []
hull_names = set()
for name, body in bodies.items():
    for g in list(body.findall('geom')):
        if g.get('mesh') is None:
            continue
        if g.get('class') == 'visual':
            visual.append({'body': name, 'mesh': g.get('mesh'), 'file': mesh_file[g.get('mesh')],
                           'pos': [float(x) for x in g.get('pos', '0 0 0').split()],
                           'quat': [float(x) for x in g.get('quat', '1 0 0 0').split()]})
            body.remove(g)
        elif g.get('class') == 'collision':
            hull_names.add(g.get('mesh'))
            g.set('mesh', g.get('mesh') + '_hull')
asset = root.find('asset')
for m in list(asset.findall('mesh')):
    asset.remove(m)
for name in sorted(hull_names):
    ET.SubElement(asset, 'mesh', {'name': f'{name}_hull', 'file': f'hulls/{name}.STL'})

# Collision hulls first (trimesh/qhull, so scipy is needed): nothing is written until every hull exists.
# MuJoCo would compute the same hull from the full mesh at every load.
import trimesh
hull_meshes = {name: trimesh.load_mesh(src_dir / 'assets' / mesh_file[name]).convex_hull for name in sorted(hull_names)}

ET.indent(tree, space='  ')
out.parent.mkdir(parents=True, exist_ok=True)
header = ('<!-- Generated by scripts/build-g1-mjcf.py from MuJoCo Menagerie unitree_g1/g1_with_hands.xml '
          f'(commit {menagerie_commit}, BSD-3-Clause, see LICENSE-g1). Do not edit by hand. -->\n')
out.write_text(header + ET.tostring(root, encoding='unicode') + '\n')

# Visual meshes at full resolution, from the same Menagerie revision, plus the renderer's manifest; stale
# files from an earlier revision are removed so nothing orphaned gets deployed
meshes = repo / 'public' / 'models' / 'meshes'
meshes.mkdir(parents=True, exist_ok=True)
visual_files = sorted({v['file'] for v in visual})
for f in visual_files:
    shutil.copy(src_dir / 'assets' / f, meshes / f)
for stale in meshes.glob('*.STL'):
    if stale.name not in visual_files:
        stale.unlink()
(meshes / 'visual.json').write_text('{"comment": "generated by scripts/build-g1-mjcf.py: visual meshes hanging off robot bodies, pos/quat (w,x,y,z) in the body frame",\n "geoms": [\n'
                                    + ',\n'.join('  ' + json.dumps(v) for v in visual) + '\n ]}\n')
print('copied', len(visual_files), 'visual meshes,', len(visual), 'visual geoms')

hulls = meshes / 'hulls'
hulls.mkdir(exist_ok=True)
for stale in hulls.glob('*.STL'):
    if stale.stem not in hull_names:
        stale.unlink()
for name, hull in hull_meshes.items():
    hull.export(hulls / f'{name}.STL')
print('wrote', len(hull_meshes), 'collision hulls,', sum(len(h.faces) for h in hull_meshes.values()), 'faces')
shutil.copy(src_dir / 'LICENSE', out.parent / 'LICENSE-g1')
print('wrote', out.relative_to(repo))
