#!/usr/bin/env python3
"""Generate public/mujoco/conveyor.xml and src/sim/tasks/conveyor.layout.js (the numbers the task module needs),
so scene and task never disagree.

Layout after Figure's 24-hour logistics demo, in the robot frame (x forward, y left, z up):
  - a gravity chute on the robot's left: an inclined sheet-metal slide with side rails; packages spawn at its
    top and slide/tumble down on their own
  - one continuous flat work plate in front of the robot, from the chute's foot to the output belt, with no
    gap a package could fall into
  - a driven roller belt on the robot's right carrying packages away (3 cm roller pitch so a package always
    rests on at least two rollers; packages balanced on one roller seesaw and stall)
"""
import json
import math
from pathlib import Path

TOP = 0.79                     # work-surface height
# plate: in front of the robot, from the belt edge to the chute foot
PLATE_X = (0.15, 0.55)
PLATE_Y = (-0.12, 0.38)
PLATE_THICK = 0.02
# chute: starts at the plate's left edge and rises away from the robot's left side
CHUTE_FOOT_Y = PLATE_Y[1]
CHUTE_LEN_Y = 0.6              # horizontal run
CHUTE_RISE = 0.22              # height gained over the run (~20 degrees)
CHUTE_HALF_W = 0.15            # half width (x)
CHUTE_THICK = 0.015
CHUTE_X = 0.32                 # chute centre line (where packages come to rest in x), within the left hand's reach
SPAWN_BACK = 0.08              # packages appear this far (horizontally) below the top end of the chute
# output belt
R = 0.014                      # roller radius
PITCH = 0.03
BELT_X = 0.305                 # belt centre line
HALF_LEN = 0.12                # roller half length (belt width 0.24)
ZC = TOP - R
N = 25                         # rollers (0.75 m)
OUT_Y0 = PLATE_Y[0] - 0.02     # first roller, right next to the plate edge (both run toward -y)
EXIT_Y = OUT_Y0 - PITCH * (N - 1) + 0.02  # delivered just before riding off the last roller
SIZES = [(0.06, 0.04, 0.03), (0.075, 0.05, 0.025), (0.04, 0.04, 0.04)]  # package half extents, two of each

ANGLE = math.atan2(CHUTE_RISE, CHUTE_LEN_Y)


def f(x):
    """Format a number without float noise."""
    return f'{x:.4f}'.rstrip('0').rstrip('.')


def chute():
    # the slide's top surface runs from (foot, TOP) to (foot + len, TOP + rise); the box sits under that line
    length = math.hypot(CHUTE_LEN_Y, CHUTE_RISE)
    mid_y, mid_z = CHUTE_FOOT_Y + CHUTE_LEN_Y / 2, TOP + CHUTE_RISE / 2
    ny, nz = -math.sin(ANGLE), math.cos(ANGLE)          # surface normal (points up and toward the foot)
    cy, cz = mid_y - ny * CHUTE_THICK / 2, mid_z - nz * CHUTE_THICK / 2
    q = f'{math.cos(ANGLE / 2):.6f} {math.sin(ANGLE / 2):.6f} 0 0'  # tilt about +x: the far (+y) end is higher
    out = ['    <!-- gravity chute: low-friction sheet; priority 3 so its friction, not the parcel\'s, applies -->',
           f'    <body name="chute" pos="{CHUTE_X} 0 0">',
           f'      <geom name="chute_slide" type="box" size="{CHUTE_HALF_W} {f(length / 2)} {f(CHUTE_THICK / 2)}" pos="0 {f(cy)} {f(cz)}" quat="{q}" '
           'solref="0.01 1" priority="3" friction="0.25 0.005 0.0001" material="chute"/>']
    rail_h = 0.05
    for sx in (-1, 1):
        ry, rz = mid_y + ny * (CHUTE_THICK / 2 + rail_h / 2 - CHUTE_THICK), mid_z + nz * (rail_h / 2 - CHUTE_THICK / 2)
        out.append(f'      <geom name="chute_rail{0 if sx < 0 else 1}" type="box" size="0.008 {f(length / 2)} {f(rail_h / 2)}" '
                   f'pos="{f(sx * (CHUTE_HALF_W + 0.008))} {f(ry)} {f(rz)}" quat="{q}" solref="0.01 1" priority="3" friction="0.25" material="rail"/>')
    # legs under the far end
    for sx in (-1, 1):
        top_z = TOP + CHUTE_RISE - CHUTE_THICK
        out.append(f'      <geom type="box" size="0.012 0.02 {f(top_z / 2)}" pos="{f(sx * (CHUTE_HALF_W - 0.02))} {f(CHUTE_FOOT_Y + CHUTE_LEN_Y - 0.03)} {f(top_z / 2)}" material="leg"/>')
    out.append('    </body>')
    return out


def plate():
    cx, cy = (PLATE_X[0] + PLATE_X[1]) / 2, (PLATE_Y[0] + PLATE_Y[1]) / 2
    hx, hy = (PLATE_X[1] - PLATE_X[0]) / 2, (PLATE_Y[1] - PLATE_Y[0]) / 2
    out = ['    <!-- flat work plate: sheet metal on a frame -->',
           f'    <body name="plate" pos="{f(cx)} {f(cy)} 0">',
           f'      <geom name="plate_top" type="box" size="{f(hx)} {f(hy)} {f(PLATE_THICK / 2)}" pos="0 0 {f(TOP - PLATE_THICK / 2)}" '
           'class="touchable" material="plate"/>',
           # a low lip on the far edge keeps fast parcels from shooting off the back
           f'      <geom name="plate_lip" type="box" size="0.006 {f(hy)} 0.015" pos="{f(hx - 0.006)} 0 {f(TOP + 0.015)}" class="touchable" material="rail"/>']
    for sx in (-1, 1):
        for sy in (-1, 1):
            out.append(f'      <geom type="box" size="0.02 0.02 {f((TOP - PLATE_THICK) / 2)}" pos="{f(sx * (hx - 0.03))} {f(sy * (hy - 0.03))} {f((TOP - PLATE_THICK) / 2)}" material="leg"/>')
    out.append('    </body>')
    return out


def belt(name, y0, direction):
    out = [f'    <!-- {name}: {N} driven rollers, surface at z={TOP}, running toward the robot\'s right (-y) -->']
    ya, yb = sorted([y0, y0 + direction * PITCH * (N - 1)])
    L = (yb - ya) / 2 + PITCH / 2
    yc = (ya + yb) / 2
    out.append(f'    <body name="{name}_frame" pos="{BELT_X} 0 0">')
    for sx in (-1, 1):
        out.append(f'      <geom class="touchable" type="box" size="0.012 {L:.3f} 0.035" pos="{sx * (HALF_LEN + 0.015):.3f} {yc:.3f} {ZC + 0.01:.3f}" material="rail"/>')
        for ly in (ya, yb):
            # legs sit under the rails, outside the roller length (rollers must not touch the frame)
            out.append(f'      <geom type="box" size="0.012 0.02 {(ZC - 0.025) / 2:.3f}" pos="{sx * (HALF_LEN + 0.015):.3f} {ly:.3f} {(ZC - 0.025) / 2:.3f}" material="leg"/>')
    out.append('    </body>')
    for i in range(N):
        y = y0 + direction * PITCH * i
        out.append(f'    <body name="{name}_roller{i}" pos="{BELT_X} {y:.3f} {f(ZC)}">')
        out.append(f'      <joint name="{name}_roller{i}" type="hinge" axis="1 0 0" damping="0.001"/>')
        out.append(f'      <geom class="roller" name="{name}_roller{i}"/>')
        out.append('    </body>')
    return out


def motors(name):
    return [f'    <velocity name="{name}_roller{i}" joint="{name}_roller{i}" kv="0.3" ctrlrange="-15 15"/>' for i in range(N)]


pkgs = []
for k, (hx, hy, hz) in enumerate(SIZES):
    for j in range(2):
        n = 2 * k + j
        # the shipping label is a printed face, not an object: a visual-only, paper-thin geom the renderer
        # textures (task.geometry); it has no collision and no mass
        pkgs.append(f'''    <body name="package{n}" pos="{-3 - 0.3 * n:.1f} 0 {f(hz + 0.001)}">
      <freejoint name="package{n}_free"/>
      <geom name="package{n}" class="object" type="box" size="{hx} {hy} {hz}" mass="0.4" condim="4" friction="0.7 0.01 0.0005" material="cardboard"/>
      <geom name="label{n}" class="label" size="{f(min(hx * 0.85, 0.05))} {f(min(hy * 0.85, 0.035))} 0.0002" pos="0 0 {f(hz + 0.0002)}" material="label"/>
    </body>''')

# spawn point on the chute surface, just below its top end; the task lifts each package along the surface
# normal by its own rotated extent so no face starts inside the sheet
spawn_y = CHUTE_FOOT_Y + CHUTE_LEN_Y - SPAWN_BACK
spawn_z = TOP + CHUTE_RISE * (spawn_y - CHUTE_FOOT_Y) / CHUTE_LEN_Y
chute_normal = [0, round(-math.sin(ANGLE), 6), round(math.cos(ANGLE), 6)]

xml = f'''<!--
  Conveyor package handling after Figure's 24-hour logistics demo: packages slide down a chute on the robot's
  left onto a flat work plate; the operator turns each one shipping-label up and sets it on the roller belt on
  the right, which carries it away. Robot faces +x, +y is the robot's left, z is up.
  Generated by scripts/build-conveyor-scene.py; do not edit by hand.
-->
<mujoco model="g1_conveyor">
  <include file="g1_upper.xml"/>

  <!-- timestep is overridden at load time by the app (?dt=...) -->
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"/>

  <default>
    <default class="touchable">
      <geom solref="0.01 1" priority="1"/>
    </default>
    <!-- the manipulated objects: priority 2 so their (randomized) friction applies against plate, rollers and
         hands alike (on a priority tie MuJoCo takes the larger friction, which would hide the randomization) -->
    <default class="object">
      <geom solref="0.01 1" priority="2"/>
    </default>
    <default class="roller">
      <geom type="cylinder" size="{R} {HALF_LEN}" quat="0.7071068 0 0.7071068 0" condim="3" friction="0.9 0.005 0.0001" solref="0.01 1" priority="1" material="roller"/>
    </default>
    <default class="label">
      <geom type="box" contype="0" conaffinity="0" density="0"/>
    </default>
  </default>

  <asset>
    <material name="floor" rgba="0.42 0.42 0.44 1"/>
    <material name="roller" rgba="0.3 0.31 0.33 1"/>
    <material name="rail" rgba="0.55 0.57 0.6 1"/>
    <material name="leg" rgba="0.35 0.36 0.38 1"/>
    <material name="plate" rgba="0.72 0.74 0.76 1"/>
    <material name="chute" rgba="0.66 0.68 0.7 1"/>
    <material name="cardboard" rgba="0.62 0.47 0.3 1"/>
    <material name="label" rgba="0.95 0.95 0.92 1"/>
  </asset>

  <worldbody>
    <geom name="floor" type="plane" size="0 0 0.05" material="floor"/>

''' + "\n".join(chute()) + '''

''' + "\n".join(plate()) + '''

''' + "\n".join(belt('belt_out', OUT_Y0, -1)) + '''

''' + "\n".join(pkgs) + '''
  </worldbody>

  <actuator>
''' + "\n".join(motors('belt_out')) + '''
  </actuator>
</mujoco>
'''
repo = Path(__file__).resolve().parent.parent
out = repo / 'public' / 'mujoco' / 'conveyor.xml'
out.write_text(xml)
layout = {
    'pool': 2 * len(SIZES), 'sizes': SIZES, 'beltX': BELT_X, 'beltTop': TOP, 'rollerRadius': R,
    'spawn': [CHUTE_X, round(spawn_y, 4), round(spawn_z, 4)], 'chuteNormal': chute_normal,
    'plateX': list(PLATE_X), 'plateY': list(PLATE_Y),
    'exitY': round(EXIT_Y, 4), 'outputStartY': round(OUT_Y0 + 0.02, 4),
}
layout_path = repo / 'src' / 'sim' / 'tasks' / 'conveyor.layout.js'
layout_path.write_text('// Generated by scripts/build-conveyor-scene.py together with public/mujoco/conveyor.xml; do not edit.\n'
                       f'export const LAYOUT = {json.dumps(layout)}\n')
print('wrote', out.name, 'and', layout_path.name, f'(chute {math.degrees(ANGLE):.1f} deg, {N} rollers)')
