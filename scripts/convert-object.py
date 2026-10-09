#!/usr/bin/env python3
"""Turn a mesh (STL/OBJ/GLB) into a MuJoCo-ready object: a visual mesh plus a convex decomposition for collision,
so containers keep their cavities (MuJoCo collides meshes as convex hulls).

    python3 scripts/convert-object.py <mesh> <name> [--scale 0.001] [--pieces 12] [--threshold 0.05]

Writes public/models/objects/<name>/visual.stl and c0.stl ... cN.stl (metres, centred on the bounding-box floor
centre so pos places the object's base), and prints the MJCF snippet to paste into a scene.
Requires: pip install trimesh coacd
"""
import argparse
from pathlib import Path

import coacd
import numpy as np
import trimesh

ap = argparse.ArgumentParser()
ap.add_argument('mesh')
ap.add_argument('name')
ap.add_argument('--scale', type=float, default=0.001, help='input units to metres (default: mm)')
ap.add_argument('--pieces', type=int, default=12)
ap.add_argument('--threshold', type=float, default=0.05)
args = ap.parse_args()

mesh = trimesh.load(args.mesh, force='mesh')
mesh.apply_scale(args.scale)
lo, hi = mesh.bounds
mesh.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
size = mesh.bounds[1] - mesh.bounds[0]

out = Path(__file__).resolve().parent.parent / 'public' / 'models' / 'objects' / args.name
out.mkdir(parents=True, exist_ok=True)
for stale in out.glob('c*.stl'):  # a re-run with fewer pieces must not leave old ones behind
    stale.unlink()
mesh.export(out / 'visual.stl')

coacd.set_log_level('error')
parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), threshold=args.threshold, max_convex_hull=args.pieces,
                        resolution=2000, mcts_nodes=20, mcts_iterations=150, mcts_max_depth=3, merge=True, seed=1)
# CoACD returns the hulls in a run-dependent order (and their exact shape can vary slightly between machines):
# order them canonically so re-runs change as little as possible
hulls = sorted((trimesh.Trimesh(v, f) for v, f in parts), key=lambda h: (-round(h.volume, 9), *np.round(h.centroid, 6)))
for i, hull in enumerate(hulls):
    hull.export(out / f'c{i}.stl')

print(f'{args.name}: {len(mesh.faces)} faces, size {size[0]:.3f} x {size[1]:.3f} x {size[2]:.3f} m, {len(parts)} convex pieces')
print('\nMJCF (paths are relative to public/models; loadScene serves them under the robot meshdir):')
print(f'    <mesh name="{args.name}_visual" file="objects/{args.name}/visual.stl"/>')
for i in range(len(parts)):
    print(f'    <mesh name="{args.name}_c{i}" file="objects/{args.name}/c{i}.stl"/>')
print(f'    ...\n    <geom type="mesh" mesh="{args.name}_visual" contype="0" conaffinity="0" density="0" material="..."/>')
for i in range(len(parts)):
    print(f'    <geom type="mesh" mesh="{args.name}_c{i}" class="touchable" group="3"/>')
