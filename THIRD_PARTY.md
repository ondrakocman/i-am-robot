# Third-party assets and licenses

| Path | Source | License |
|---|---|---|
| `public/mujoco/g1_upper.xml` (generated), `public/models/meshes/*.STL` | [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) `unitree_g1`, derived from Unitree Robotics' G1 description | BSD-3-Clause, see `public/mujoco/LICENSE-g1` |
| `public/models/objects/bin/*` | Project author's own bin model (`assets/bin-source.stl`, converted with `scripts/convert-object.py`) | MIT (this repository) |
| `mujoco.wasm` in the deployed bundle (`@mujoco/mujoco`) | Google DeepMind, [MuJoCo](https://github.com/google-deepmind/mujoco) | Apache-2.0 ([text](https://www.apache.org/licenses/LICENSE-2.0)) |
| JavaScript bundle | three.js, React, @react-three/fiber, @react-three/xr and their dependencies; the XR emulator chunks (localhost only) include Font Awesome icons (CC-BY-4.0), @bufbuild/protobuf (Apache-2.0) and webxr-layers-polyfill (Apache-2.0) | per package; full texts are generated into `THIRD_PARTY_LICENSES.txt` next to the deployed site at build time |

Everything else in this repository is covered by `LICENSE` (MIT). This file is shipped as `THIRD_PARTY.md` next to the deployed site.
