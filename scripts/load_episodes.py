#!/usr/bin/env python3
"""Read .iamr episode files downloaded from the headset (layout in src/sim/episode.js).

    python3 scripts/load_episodes.py episodes.iamr

    from load_episodes import iter_episodes, load_episodes
    for ep in iter_episodes('episodes.iamr'):   # or load_episodes(path) for a list
        ep['header']['task'], ep['header']['outcome'], ep['action'].shape, ep['qpos'].shape

Each episode is a dict with the JSON header under 'header' and one read-only float32 array of shape
(frames, size) per recorded field, at header['control_hz'] (50 Hz):
  time      seconds since the episode started
  action    actuator targets (= what a policy should output). The first header['robot_nu'] entries are the
            robot's, in header['actuator_names'] order (waist x3, then per arm: 7 arm joints followed by 7 hand
            joints; note the two hands list their fingers in different orders). Any remaining entries are scene
            actuators (conveyor belt motors).
  qpos/qvel full simulation state incl. the free objects, named in header['qpos_names'] / ['qvel_names'];
            a free object's angular velocity (wx, wy, wz) is in the object's body frame, MuJoCo convention
  input     retargeted operator command per hand, columns named in header['input_names']: tracked, palm pos (3),
            palm quat wxyz (4), thumb rotation [-1, 1], then curls [0, 1] for thumb_1, thumb_2, index_0, index_1,
            middle_0, middle_1
  raw       viewer (head) pose (pos 3 + quat wxyz 4) then 25 WebXR joints per hand (pos 3 + quat wxyz 4), all
            in the robot frame, joint order in header['raw_layout']['hand_joints']; an untracked joint is all zeros
  touching  per hand: 1 while it touches any task object
The header also carries the layout, randomized physics, logged teleport events, initial/final state, asset
hashes and the app/engine versions needed to replay the episode exactly (see src/sim/replay.js).
"""
import json
import os
import struct
import sys

import numpy as np

MAGIC = 0x524D4149  # "IAMR"
# v1 (pre-release builds) has the same binary layout with fewer header fields; see src/sim/episode.js
FORMATS = ('iamr-episode-v1', 'iamr-episode-v2')


def iter_episodes(path):
    """Yields the episodes of a .iamr file one at a time; the frames are memory-mapped, so files of any size
    work and only the fields you touch are read."""
    with open(path, 'rb') as f:
        size = os.fstat(f.fileno()).st_size
        o = 0
        while o < size:
            start = o

            def read(n):
                nonlocal o
                b = f.read(n)
                if len(b) != n:
                    raise ValueError(f'truncated file: episode at byte {start} is incomplete')
                o += n
                return b

            magic, header_bytes = struct.unpack('<II', read(8))
            if magic != MAGIC:
                raise ValueError(f'bad magic at byte {start}')
            header = json.loads(read(header_bytes).rstrip(b'\0'))
            if header.get('format') not in FORMATS:
                raise ValueError(f"unsupported episode format {header.get('format')!r}")
            (data_bytes,) = struct.unpack('<I', read(4))
            if o + data_bytes > size:
                raise ValueError(f'truncated file: episode at byte {start} is incomplete')
            if data_bytes != header['frames'] * header['frame_size'] * 4:
                raise ValueError(f'episode {header["episode"]}: data size does not match header')
            # read-only views into the mapped file; .copy() before modifying
            frames = np.memmap(path, '<f4', 'r', o, (header['frames'], header['frame_size']))
            o += data_bytes
            f.seek(o)
            ep = {'header': header}
            for field in header['fields']:
                ep[field['name']] = frames[:, field['offset']:field['offset'] + field['size']]
            yield ep


def load_episodes(path):
    return list(iter_episodes(path))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    for path in sys.argv[1:]:
        for ep in iter_episodes(path):
            h = ep['header']
            print(f"{h['task']:12s} episode {h['episode']:4d}  {h['outcome']:9s}  {h['duration']:6.2f} s  "
                  f"{h['frames']} frames @ {h['control_hz']:.0f} Hz  action {ep['action'].shape}  qpos {ep['qpos'].shape}")
