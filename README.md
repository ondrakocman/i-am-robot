# I Am Robot

Embodied VR teleoperation of a Unitree G1 + Dex3 in the Meta Quest 3 browser, for collecting imitation-learning
demonstrations without a physical robot.

## Tube-into-box task (MuJoCo physics)

Modeled on Isaac Lab's `PickPlace-FixedBaseUpperBodyIK-G1`: the tube stands on the robot's left, the box sits on
its right. An episode succeeds when the tube rests inside the box with both hands off it for 0.5 s; then it is
saved and the scene re-randomizes. Physics is MuJoCo (official WASM build) running in a Web Worker on the headset.

```
npm install
npm run dev          # open https://<your-ip>:5173 in the Quest browser
```

- Recording starts when your hands are tracked and stops on success, drop or a 60 s timeout.
- Manual reset: hold a robot hand on the red sphere to your upper left for ~0.6 s.
- Robot fingers glow green while they touch the tube.
- The floating panel shows the episode state and whether physics keeps up (`1.00×` real time).
- Episodes are stored in the headset browser (IndexedDB). Exit VR and press **Download** to get one `.iamr` file.

URL options: `?dt=0.004` (physics timestep, default 2 ms), `?autopilot` (scripted demo, no headset needed),
`?view=eye` (desktop preview from the robot's head), `?legacy` (the original kinematic scene).

### Data

```
python3 scripts/load_episodes.py episodes.iamr
```

Per episode, at 50 Hz: `action` (the 31 actuator targets: waist, 2x7 arm, 2x7 hand), `qpos`/`qvel` (full sim
state including the tube), `input` (retargeted operator command), `raw` (head pose + all 25 WebXR joints per hand),
`touching`. The header holds the layout, seed, outcome, joint/actuator names, `initial_qpos`, the randomized
`physics` (tube mass 0.2–0.4 kg, friction 0.5–0.9), `peak_arm_velocity` and `flags` (`fast_motion` above 6 rad/s).
Each episode replays bit-for-bit from `initial_qpos` + `physics` + the logged actions with the same MuJoCo build,
so camera images can be rendered afterwards.

### Fidelity choices

- Joint PD gains are the ones Unitree's own teleop stack ([xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate))
  sends to the real G1: shoulder/elbow kp=80 kd=3, wrist kp=40 kd=1.5, waist kp=300 kd=3, Dex3 kp=1.5 kd=0.2.
  Actuator force limits come from Unitree's model via MuJoCo Menagerie. Gravity is compensated on the arms.
- Anti-windup: the commanded joint target may lead the measured joint by at most 0.12 rad, so a hand blocked by
  the table pushes with a bounded force and does not whip when it comes free.
- Contacts on the tube, table and box use `solref="0.01 1"` (stiffer than MuJoCo's default 0.02), valid for all
  supported timesteps (`?dt=` up to 0.005).

### Development

- `npm run sim-check`: runs the scripted pick-and-place headlessly in Node, checks success and exact replay, and
  reports the step cost.
- `src/sim/TubeBoxSim.js`: task, IK → joint targets, finger retargeting, recording (shared by the worker and Node).
- `public/mujoco/g1_upper.xml` is generated from MuJoCo Menagerie by `scripts/build-g1-mjcf.py` (fixed base, legs
  as static visuals, gravity compensation, palm/grip/eye sites).
- `public/mujoco/tube_box.xml`: the task scene.
