#!/usr/bin/env python3
"""Read .iamr episode files downloaded from the headset (layout in src/sim/episode.js).

    python3 scripts/load_episodes.py episodes.iamr

    from load_episodes import load_episodes
    for ep in load_episodes('episodes.iamr'):
        ep['header']['task'], ep['header']['outcome'], ep['action'].shape, ep['qpos'].shape

Each episode is a dict with the JSON header under 'header' and one float32 array of shape (frames, size) per
recorded field, at header['control_hz'] (50 Hz):
  time      seconds since the episode started
  action    actuator targets (= what a policy should output). The first header['robot_nu'] entries are the
            robot's, in header['actuator_names'] order (waist x3, then per arm: 7 arm joints followed by 7 hand
            joints; note the two hands list their fingers in different orders). Any remaining entries are scene
            actuators (conveyor belt motors).
  qpos/qvel full simulation state incl. the free objects, named in header['qpos_names'] / ['qvel_names']
  input     retargeted operator command per hand: tracked, palm pos (3), palm quat wxyz (4), 7 finger commands
  raw       viewer (head) pose (pos 3 + quat wxyz 4) then 25 WebXR joints per hand (pos 3 + quat wxyz 4), all
            in the robot frame; an untracked joint is all zeros
  touching  per hand: 1 while it touches any task object
The header also carries the layout, randomized physics, logged teleport events, initial/final state, asset
hashes and the app/engine versions needed to replay the episode exactly (see src/sim/replay.js).
"""
import json
import struct
import sys

import numpy as np

MAGIC = 0x524D4149  # "IAMR"
FORMAT = 'iamr-episode-v1'


def load_episodes(path):
    with open(path, 'rb') as f:
        data = f.read()
    episodes, o = [], 0
    while o < len(data):
        magic, header_bytes = struct.unpack_from('<II', data, o)
        if magic != MAGIC:
            raise ValueError(f'bad magic at byte {o}')
        header = json.loads(data[o + 8:o + 8 + header_bytes].rstrip(b'\0'))
        if header.get('format') != FORMAT:
            raise ValueError(f"unsupported episode format {header.get('format')!r}")
        o += 8 + header_bytes
        (data_bytes,) = struct.unpack_from('<I', data, o)
        o += 4
        frames = np.frombuffer(data, '<f4', data_bytes // 4, o).reshape(-1, header['frame_size'])
        o += data_bytes
        ep = {'header': header}
        for field in header['fields']:
            ep[field['name']] = frames[:, field['offset']:field['offset'] + field['size']]
        episodes.append(ep)
    return episodes


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    for path in sys.argv[1:]:
        for ep in load_episodes(path):
            h = ep['header']
            print(f"{h['task']:12s} episode {h['episode']:4d}  {h['outcome']:9s}  {h['duration']:6.2f} s  "
                  f"{h['frames']} frames @ {h['control_hz']:.0f} Hz  action {ep['action'].shape}  qpos {ep['qpos'].shape}")
