# I Am Robot

Browser-based VR teleoperation of a Unitree G1 humanoid with Dex3 hands on a Meta Quest 3, for collecting
imitation-learning demonstrations without a physical robot. Physics is MuJoCo (the official WebAssembly build)
running in a Web Worker on the headset: a URL is the whole install, nothing leaves the device, and every episode
replays bit-for-bit from its log.

Live: https://ondrakocman.github.io/i-am-robot/

## How it works

- You stand where the robot stands and see its arms and hands as your own. Quest hand tracking drives a
  damped-least-squares IK to arm joint targets; thumb, index and middle finger curls are retargeted to the
  Dex3's seven joints. The targets go to position actuators with the real robot's PD gains, so grasps obey the
  real force limits and a blocked hand pushes with bounded force instead of whipping free.
- Recording starts when your hands are tracked and ends on success, drop, timeout, or when you leave VR.
  Episodes are stored in the headset browser; exit VR and press **Download** for one `.iamr` file.
- Manual reset: hold a robot hand on the red sphere to your upper left. Robot fingers glow while they touch an
  object. If the robot hand can't follow yours (blocked, out of reach), your real hand fades in as a skeleton.
- The panel behind the table shows the recording state, saved-episode count and whether physics keeps up
  (`1.00× real time`). If it drops, try `?dt=0.004`. The display runs at 90 Hz; `?hz=72` or `?hz=120` asks for
  another rate the headset offers.

## Tasks

Pick the task on the landing page (or `?task=`):

- **Tube into box** (`tube_box`), modeled on Isaac Lab's `PickPlace-FixedBaseUpperBodyIK-G1`: a hollow steel tube
  stands on the robot's left, the box sits on its right.
- **Conveyor** (`conveyor`), after Figure's 24-hour logistics demo: packages slide down a chute on the robot's
  left onto a flat work plate in front of it, in a random orientation; turn each one shipping-label up and set it
  on the roller belt on the right, which carries it away. Five packages per episode drawn from six cardboard
  boxes and two soft poly-mailer bags, printed labels (address, barcode, QR), randomized mass, friction, bag
  softness and belt speed; each package is scored `correct` / `wrong_face` / `dropped`.
- **Cube sorting** (`cube_sort`): six printed PLA cubes, two of each colour, into the matching bins (the bin is a
  real model, converted with `scripts/convert-object.py`).

Static tasks succeed when the goal holds with every object at rest and both hands off them for 0.5 s; the
conveyor task ends once every package has been scored (delivered or dropped), `success` only if all were
label-up. Episodes also end on a drop, a timeout, 5 s without tracked hands (`lost_tracking`), leaving VR or
holding a hand on the reset button (`aborted`), a headset recenter (`recentered`), a MuJoCo instability
(`unstable`, keeping the frames recorded before it), or a physics-worker exception (`error`). Then the episode is saved and the scene re-randomizes (object placement, mass, friction, belt
speed), all logged.

Adding a task: one module in `src/sim/tasks/` (scene XML, object list, `reset`/`randomize`, `goal` or `update`,
`solved` and `reachTargets` for the headless check, `materials`/`geometry` for the renderer) and a line in
`tasks/index.js`; the check script picks it up automatically.

## Running it

```
npm install
npm run dev          # https://<your-ip>:5173 in the Quest browser (self-signed certificate: accept it once)
npm run check        # lint, unit tests and the headless task checks CI runs (CI also replays the recorded file)
npm run build
```

URL options: `?task=<name>`, `?dt=0.001|0.002|0.0025|0.004|0.005` (physics timestep; control stays at 50 Hz),
`?autopilot` (scripted demo of the tube task, no headset needed), `?view=eye` (desktop preview from the robot's head).

## Data

```
python3 scripts/load_episodes.py episodes.iamr      # numpy arrays per field (see the docstring for the layout)
node scripts/replay.mjs episodes.iamr [poses.ndjson] # verifies bit-exact replay; optionally streams body poses
```

Per episode, at 50 Hz: `action` (actuator targets; the first `robot_nu` are the robot's, in `actuator_names`
order: waist ×3, then per arm 7 arm joints followed by 7 hand joints; the two hands list their fingers in
different orders, so slice by name), `qpos`/`qvel` (full simulation state including the objects), `input`
(retargeted operator command), `raw` (head pose and all 25 WebXR joints per hand, zero when untracked),
`touching`. Free-object angular velocities in `qvel` are in the object's body frame (MuJoCo convention); the
Python loader returns read-only views. The header holds the task and its language instruction, layout, seed, outcome and per-task result,
randomized `physics`, logged teleport `events`, initial and final state, `peak_arm_velocity`, MuJoCo warning
counters and `flags` (`fast_motion` above 6 rad/s, `slow_physics` if the headset fell below 0.9× real time,
`unstable`), the real-time factor during the episode, SHA-256 hashes of every model file, and the app commit
and MuJoCo version. Episodes are numbered per session; `session` + `episode` is the unique key.

Replay: `initial_qpos` + `initial_ctrl` + `physics` + actions + events reproduce `qpos`/`qvel` exactly with the
same `@mujoco/mujoco` build and the same model files (`header.assets` holds their hashes; `app_version` is the
commit to check out; `src/sim/replay.js` is the reference). A renderer needs only the logged `qpos`: body poses
follow from the scene MJCF, and the robot's visual meshes from `public/models/meshes/visual.json` (the MJCF
itself carries only collision hulls, so MuJoCo's own viewer shows no robot shell), so camera images can be
produced offline from any viewpoint with any renderer.

What is collected: robot and object state, your head pose and hand skeleton at 50 Hz, wall-clock start/end
times, the browser's user-agent string, the headset's display rate and a random session id. Nothing is
uploaded; the data stays in the headset until you download it. Hand and head motion are personal data, so get
consent before sharing datasets recorded by others.

## Fidelity choices

- **Soft parcels** (`src/sim/soft.js`): XPBD tetrahedral lattices (about 100 particles, 400 distance and 240
  volume constraints per bag) stepped twice per MuJoCo step in the same worker. They collide with MuJoCo's own
  geoms (hand links as boxes fitted to their hulls, scene boxes, cylinders, planes, the rigid parcels) with
  Coulomb friction against the surface's real velocity, so a turning roller carries a bag; the reaction goes back
  to MuJoCo bodies through `xfrc_applied` (a hand carries the bag's weight, a bag pushes a box). Particle positions
  are recorded per frame and replayed bit-exactly like the rigid state. Limits: a bag does not collide with
  itself, bag-bag contact is particle repulsion, and the reaction on rigid bodies is a damped penalty.

- Robot model from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) (Unitree's G1
  description, BSD-3), fixed base, legs static. Generated by `scripts/build-g1-mjcf.py`. Physics collides with
  precomputed convex hulls of the Menagerie meshes (what MuJoCo would compute from them anyway, at a fraction of
  the memory); the renderer loads the full-resolution meshes (630k triangles) itself from
  `public/models/meshes/visual.json`.
- Joint PD gains are the ones Unitree's own teleop stack ([xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate))
  sends to the real G1: shoulder/elbow kp=80 kd=3, wrist kp=40 kd=1.5, waist kp=300 kd=3, Dex3 kp=1.5 kd=0.2.
  Actuator force limits come from Unitree's model. Gravity is compensated on the arms.
- Dex3 finger joints use frictionloss 0.01 and armature 0.0005 instead of Menagerie's body-wide 0.3 / 0.01: with
  kp=1.5 the original friction swallowed every command below ~0.2 rad.
- Anti-windup: a commanded joint target may lead the measured joint by at most 0.12 rad.
- Contacts on objects, table and containers use `solref="0.01 1"` (stiffer than MuJoCo's default 0.02), valid for
  every supported timestep. Actions are stored as float32 and applied as float32, which is what makes replay exact.

## Repository

- `src/sim/TaskSim.js` IK, finger retargeting, task loop, recording (runs in the worker and in Node)
- `src/sim/sim.worker.js` real-time stepping off the render thread; `src/components/MujocoScene.jsx` rendering,
  hand tracking, calibration, HUD
- `src/sim/tasks/` task modules; `public/mujoco/` scenes; `public/models/` meshes
- `scripts/sim-check.mjs` headless gate (task success, bit-exact replay, instability handling, goal
  reachability, reset validity); `scripts/unit-tests.mjs` (frame conversions, retargeting, file round trip);
  `scripts/replay.mjs`; `scripts/load_episodes.py`
- `zustand` is a dependency only because the XR emulator's dev UI (`@iwer/devui`, localhost only) imports it
  without declaring it; the app itself does not use it
- `scripts/build-g1-mjcf.py`, `scripts/build-conveyor-scene.py` and `scripts/convert-object.py` regenerate the
  generated assets (`pip install -r requirements-tools.txt`, Python ≥ 3.11); `assets/` holds source models that are
  not served.
  The bin's colliders are hand-written boxes at the model's true wall thickness (`public/mujoco/cube_sort.xml`);
  `convert-object.py --hulls` also writes CoACD convex pieces for objects that need them

License: MIT (see `LICENSE`); third-party assets in `THIRD_PARTY.md`.
