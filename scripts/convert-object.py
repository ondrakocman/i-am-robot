#!/usr/bin/env python3
"""Turn a mesh (STL/OBJ/GLB) into a MuJoCo-ready visual object: public/models/objects/<name>/visual.stl in metres,
centred on the bounding-box floor centre so `pos` places the object's base, plus the MJCF snippet to paste.

    python3 scripts/convert-object.py <mesh> <name> [--scale 0.001] [--hulls [--pieces 12] [--threshold 0.05]]

Collision is a separate decision. Simple shapes (bins, boxes, tubes) get hand-written box geoms at the true wall
thickness (see cube_sort.xml): no seams for a fingertip to slip through, and exactly what the operator sees.
`--hulls` additionally writes a CoACD convex decomposition (c0.stl ... cN.stl) for shapes that need one; its
pieces can differ slightly between machines, which changes the asset hashes in new episodes.
Requires: pip install -r requirements-tools.txt
"""
import argparse
from pathlib import Path

import numpy as np
import trimesh

ap = argparse.ArgumentParser()
ap.add_argument('mesh')
ap.add_argument('name')
ap.add_argument('--scale', type=float, default=0.001, help='input units to metres (default: mm)')
ap.add_argument('--hulls', action='store_true', help='also write a CoACD convex decomposition for collision')
ap.add_argument('--pieces', type=int, default=12, help='--hulls: maximum number of convex pieces')
ap.add_argument('--threshold', type=float, default=0.05, help='--hulls: CoACD concavity threshold')
args = ap.parse_args()

mesh = trimesh.load(args.mesh, force='mesh')
mesh.apply_scale(args.scale)
lo, hi = mesh.bounds
mesh.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
size = mesh.bounds[1] - mesh.bounds[0]

out = Path(__file__).resolve().parent.parent / 'public' / 'models' / 'objects' / args.name
out.mkdir(parents=True, exist_ok=True)
for stale in out.glob('c*.stl'):  # a re-run (with fewer pieces, or without --hulls) must not leave old ones behind
    stale.unlink()
mesh.export(out / 'visual.stl')

hulls = []
if args.hulls:
    import coacd
    coacd.set_log_level('error')
    parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), threshold=args.threshold, max_convex_hull=args.pieces,
                            resolution=2000, mcts_nodes=20, mcts_iterations=150, mcts_max_depth=3, merge=True, seed=1)
    # CoACD returns the hulls in a run-dependent order (and their exact shape can vary slightly between machines):
    # order them canonically so re-runs change as little as possible
    hulls = sorted((trimesh.Trimesh(v, f) for v, f in parts), key=lambda h: (-round(h.volume, 9), *np.round(h.centroid, 6)))
    for i, hull in enumerate(hulls):
        hull.export(out / f'c{i}.stl')

print(f'{args.name}: {len(mesh.faces)} faces, size {size[0]:.3f} x {size[1]:.3f} x {size[2]:.3f} m'
      + (f', {len(hulls)} convex pieces' if args.hulls else ''))
print('\nMJCF (paths are relative to public/models; loadScene serves them under the robot meshdir):')
print(f'    <mesh name="{args.name}_visual" file="objects/{args.name}/visual.stl"/>')
for i in range(len(hulls)):
    print(f'    <mesh name="{args.name}_c{i}" file="objects/{args.name}/c{i}.stl"/>')
print(f'    ...\n    <geom type="mesh" mesh="{args.name}_visual" contype="0" conaffinity="0" density="0" material="..."/>')
if hulls:
    for i in range(len(hulls)):
        print(f'    <geom type="mesh" mesh="{args.name}_c{i}" class="touchable" group="3"/>')
else:
    print('    <!-- collision: box geoms (class="touchable" group="3") matching the walls and floor, see cube_sort.xml -->')
