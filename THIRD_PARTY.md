# Third-party assets and licenses

| Path | Source | License |
|---|---|---|
| `public/mujoco/g1_upper.xml` (generated), `public/models/meshes/*.STL` | [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) `unitree_g1`, derived from Unitree Robotics' G1 description | BSD-3-Clause, see `public/mujoco/LICENSE-g1` |
| `public/models/objects/bin/*` | Project author's own bin model (`source.stl`, converted with `scripts/convert-object.py`) | MIT (this repository) |
| `mujoco.wasm` in the deployed bundle (`@mujoco/mujoco`) | Google DeepMind, [MuJoCo](https://github.com/google-deepmind/mujoco) | Apache-2.0 |
| JavaScript bundle | three.js, React, @react-three/fiber, @react-three/xr (and their dependencies) | MIT |

Everything else in this repository is covered by `LICENSE` (MIT). This file is shipped as `THIRD_PARTY.md` next to the deployed site.
