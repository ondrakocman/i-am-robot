# I Am Robot

Embodied VR teleoperation of a Unitree G1 + Dex3 in the Meta Quest 3 browser, for collecting imitation-learning
demonstrations without a physical robot.

## Tasks (MuJoCo physics)

Physics is MuJoCo (official WASM build) running in a Web Worker on the headset. Pick the task on the landing page
(or `?task=`):

- **Tube into box** (`tube_box`), modeled on Isaac Lab's `PickPlace-FixedBaseUpperBodyIK-G1`: a hollow steel tube
  stands on the robot's left, the box sits on its right.
- **Conveyor** (`conveyor`), in the spirit of the humanoid logistics demos: packages arrive on the left roller belt
  in a random orientation; turn each one shipping-tag up and set it on the right belt, which carries it away.
  Five packages per episode, three sizes, randomized mass/friction/belt speed. The header scores each package
  (`correct` / `wrong_face` / `dropped`).
- **Cube sorting** (`cube_sort`): six printed PLA cubes, two of each colour, into the matching bins (your own bin
  model, converted with `scripts/convert-object.py`).

Static tasks succeed when the goal holds with every object at rest and both hands off them for 0.5 s; the
conveyor task ends when all packages have left the output belt. Then the episode is saved and the scene
re-randomizes.

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
state including the objects), `input` (retargeted operator command), `raw` (head pose + all 25 WebXR joints per
hand), `touching`. The header holds the task, layout, seed, outcome, joint/actuator names, `initial_qpos`, the
randomized `physics` per object (mass, friction), `peak_arm_velocity` and `flags` (`fast_motion` above 6 rad/s).
Dynamic tasks also log `events` (object teleports such as package spawns, tagged with the frame they follow) and
`initial_ctrl` (belt motors); `robot_nu` says how many leading entries of `action` are the robot's. Each episode
replays bit-for-bit from `initial_qpos` + `initial_ctrl` + `physics` + actions + events with the same MuJoCo build,
so camera images can be rendered afterwards.

### Fidelity choices

- Joint PD gains are the ones Unitree's own teleop stack ([xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate))
  sends to the real G1: shoulder/elbow kp=80 kd=3, wrist kp=40 kd=1.5, waist kp=300 kd=3, Dex3 kp=1.5 kd=0.2.
  Actuator force limits come from Unitree's model via MuJoCo Menagerie. Gravity is compensated on the arms.
- Anti-windup: the commanded joint target may lead the measured joint by at most 0.12 rad, so a hand blocked by
  the table pushes with a bounded force and does not whip when it comes free.
- Contacts on the objects, table and containers use `solref="0.01 1"` (stiffer than MuJoCo's default 0.02), valid for all
  supported timesteps (`?dt=` up to 0.005).

### Development

- `npm run sim-check [task] [dt=0.002] [n=3]`: headless check in Node. Tasks with a scripted demonstration
  (`tube_box`) run it and must succeed; others teleport the objects into the solved configuration and goal
  detection must fire. Both verify bit-exact replay (including logged spawn/teleport events) and report the step
  cost. The scripted hand only manages round objects: the open Dex3 thumb blocks a sideways approach to a box, and
  a pitched-down palm is 2–4 cm off everywhere on this table for the G1 arm.
- `scripts/convert-object.py <mesh> <name>`: turns an STL/OBJ/GLB into a visual mesh plus a CoACD convex
  decomposition for collision (containers keep their cavities), under `public/models/objects/<name>/`.
- `scripts/build-conveyor-scene.py`: generates the conveyor scene (roller pitch, package pool).
- `src/sim/TaskSim.js`: IK → joint targets, finger retargeting, task loop, recording (shared by the worker and Node).
- `src/sim/tasks/*.js`: one module per task (scene, objects, reset/randomization, goal, optional scripted demo).
- `public/mujoco/g1_upper.xml` is generated from MuJoCo Menagerie by `scripts/build-g1-mjcf.py` (fixed base, legs
  as static visuals, gravity compensation, real PD gains, palm/grip/eye sites).
- `public/mujoco/*.xml`: task scenes.
