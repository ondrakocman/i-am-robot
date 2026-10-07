// Tube into box, modeled on Isaac Lab's PickPlace-FixedBaseUpperBodyIK-G1: object on the robot's left,
// container on its right.
import { pickPlaceKeys, sampleKeys } from '../autopilot.js'

const TUBE_HOME = [0.31, 0.15, 0.861]
const JITTER = 0.025
const BOX_INNER = [0.09, 0.1]   // half extents of the box interior (x, y)
const BOX_MAX_Z = 0.89          // tube center must be below this (box rim is at 0.86)

export default {
  name: 'tube_box',
  instruction: 'Put the steel tube into the blue box',
  title: 'Put the tube in the box',
  scene: 'mujoco/tube_box.xml',
  objects: ['tube'],
  dropZ: 0.68,
  timeout: 60,

  randomize(rng) {
    const u = (lo, hi) => lo + (hi - lo) * rng()
    return { tube: { mass: u(0.2, 0.4), friction: u(0.5, 0.9) } }
  },

  reset(sim, rng) {
    const jitter = () => (rng() * 2 - 1) * JITTER
    const tube = [TUBE_HOME[0] + jitter(), TUBE_HOME[1] + jitter(), TUBE_HOME[2]]
    sim.placeObject(0, tube)
    const b = 3 * sim.bodyId('box')
    return { tube, box: Array.from(sim.m.body_pos.slice(b, b + 3)) }
  },

  goal(sim) {
    const [x, y, z] = sim.objectPos(0)
    const [bx, by] = sim.layout.box
    return Math.abs(x - bx) < BOX_INNER[0] && Math.abs(y - by) < BOX_INNER[1] && z < BOX_MAX_Z
  },

  // Left hand carries the tube across to the box
  autopilot(sim, t) {
    const { tube, box } = sim.layout
    const grip = tube[2] + 0.02
    const keys = pickPlaceKeys(0, sim.readyGrip[0], [tube[0], tube[1], grip], [box[0] - 0.01, box[1] + 0.06, grip])
    return [sampleKeys(keys, t), null]
  },
}
