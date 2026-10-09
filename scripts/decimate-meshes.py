#!/usr/bin/env python3
"""Decimate the robot meshes in public/models/meshes in place for the Quest 3.

Unitree's STLs total ~630k triangles (a fingertip alone is 30k). The headset renders them twice per frame plus
once for the shadow pass, and MuJoCo builds collision hulls from the same files, so both get lighter.

    python3 scripts/decimate-meshes.py [--hand 2500] [--body 6000]

Quadric decimation via trimesh + fast-simplification (see requirements.txt). Idempotent: files already under
the target are left alone. Run after scripts/build-g1-mjcf.py, which copies the full-resolution files.
"""
import argparse
from pathlib import Path

import trimesh

ap = argparse.ArgumentParser()
ap.add_argument('--hand', type=int, default=2500, help='max faces per hand link')
ap.add_argument('--body', type=int, default=6000, help='max faces per other link')
args = ap.parse_args()

mesh_dir = Path(__file__).resolve().parent.parent / 'public' / 'models' / 'meshes'
before = after = 0
for path in sorted(mesh_dir.glob('*.STL')):
    mesh = trimesh.load(path, force='mesh')
    target = args.hand if '_hand_' in path.name else args.body
    before += len(mesh.faces)
    if len(mesh.faces) <= target:
        after += len(mesh.faces)
        continue
    simplified = mesh.simplify_quadric_decimation(face_count=target)
    simplified.export(path)
    after += len(simplified.faces)
    print(f'{path.name}: {len(mesh.faces)} -> {len(simplified.faces)} faces')
print(f'total {before} -> {after} faces')
