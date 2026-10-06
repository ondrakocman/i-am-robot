#!/usr/bin/env python3
"""Read .iamr episode files downloaded from the headset (see src/sim/episode.js for the layout).

    python3 scripts/load_episodes.py tube_box_2026-10-06.iamr

    from load_episodes import load_episodes
    for ep in load_episodes('tube_box.iamr'):
        ep['header']['outcome'], ep['action'].shape, ep['qpos'].shape

Each episode is a dict with the JSON header under 'header' and one float32 array of shape
(frames, size) per recorded field: time, action (actuator targets = what the policy should output),
qpos, qvel (full sim state incl. the tube's free joint), input (retargeted operator command),
raw (head pose + 25 WebXR joints per hand, MuJoCo frame), touching (left/right hand on the tube).
"""
import json
import struct
import sys

import numpy as np

MAGIC = 0x524D4149  # "IAMR"


def load_episodes(path):
    data = open(path, 'rb').read()
    episodes, o = [], 0
    while o < len(data):
        magic, header_bytes = struct.unpack_from('<II', data, o)
        if magic != MAGIC:
            raise ValueError(f'bad magic at byte {o}')
        header = json.loads(data[o + 8:o + 8 + header_bytes].rstrip(b'\0'))
        o += 8 + header_bytes
        (data_bytes,) = struct.unpack_from('<I', data, o)
        o += 4
        frames = np.frombuffer(data, '<f4', data_bytes // 4, o).reshape(-1, header['frame_size'])
        o += data_bytes
        ep = {'header': header}
        for f in header['fields']:
            ep[f['name']] = frames[:, f['offset']:f['offset'] + f['size']]
        episodes.append(ep)
    return episodes


if __name__ == '__main__':
    for path in sys.argv[1:]:
        for ep in load_episodes(path):
            h = ep['header']
            print(f"episode {h['episode']:4d}  {h['outcome']:8s}  {h['duration']:6.2f} s  "
                  f"{h['frames']} frames @ {h['control_hz']:.0f} Hz  action {ep['action'].shape}  qpos {ep['qpos'].shape}")
