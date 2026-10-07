// Package sorting, in the spirit of the humanoid logistics demos: parcels arrive in the middle of the table,
// red labels go to the left bin, blue labels to the right bin. Parcels on the far side of the body have to be
// reached across for or handed over.

const PARCELS = [
  { name: 'parcel0', color: 'red' }, { name: 'parcel1', color: 'red' },
  { name: 'parcel2', color: 'blue' }, { name: 'parcel3', color: 'blue' },
]
const SLOTS = [[0.27, 0.05], [0.34, 0.05], [0.27, -0.05], [0.34, -0.05]]
const PARCEL_Z = 0.841          // half height 0.05 on the 0.79 table top, plus a hair
const JITTER = 0.01
const YAW_JITTER = 0.3
const BIN = { red: 'bin_left', blue: 'bin_right' }
const BIN_INNER = [0.09, 0.05]  // half extents of the bin interior (x, y), a little inside the walls
const BIN_MAX_Z = 0.9           // parcel center must be below this whether it stands or lies (rim is at 0.84)

function shuffle(arr, rng) {
  for (let i = arr.length - 1; i > 0; i--) {
    const j = Math.floor(rng() * (i + 1));
    [arr[i], arr[j]] = [arr[j], arr[i]]
  }
  return arr
}

export default {
  name: 'package_sort',
  instruction: 'Sort the parcels: red labels into the left bin, blue into the right bin',
  title: 'Sort parcels: red left, blue right',
  scene: 'mujoco/package_sort.xml',
  objects: PARCELS.map(p => p.name),
  dropZ: 0.68,
  timeout: 90,

  randomize(rng) {
    const u = (lo, hi) => lo + (hi - lo) * rng()
    return Object.fromEntries(PARCELS.map(p => [p.name, { mass: u(0.1, 0.3), friction: u(0.6, 0.9) }]))
  },

  reset(sim, rng) {
    const slots = shuffle(SLOTS.slice(), rng)
    const parcels = PARCELS.map((p, i) => {
      const [sx, sy] = slots[i]
      const pos = [sx + (rng() * 2 - 1) * JITTER, sy + (rng() * 2 - 1) * JITTER, PARCEL_Z]
      const yaw = (rng() * 2 - 1) * YAW_JITTER
      sim.placeObject(i, pos, yaw)
      return { name: p.name, color: p.color, pos, yaw }
    })
    const bins = {}
    for (const [color, body] of Object.entries(BIN)) {
      const b = 3 * sim.bodyId(body)
      bins[color] = Array.from(sim.m.body_pos.slice(b, b + 3))
    }
    return { parcels, bins }
  },

  goal(sim) {
    return PARCELS.every((p, i) => {
      const [x, y, z] = sim.objectPos(i)
      const [bx, by] = sim.layout.bins[p.color]
      return Math.abs(x - bx) < BIN_INNER[0] && Math.abs(y - by) < BIN_INNER[1] && z < BIN_MAX_Z
    })
  },

  // Goal configuration for the headless check: parcels standing in their bins, two per bin
  solved(sim) {
    const count = { red: 0, blue: 0 }
    PARCELS.forEach((p, i) => {
      const [bx, by, bz] = sim.layout.bins[p.color]
      sim.placeObject(i, [bx + (count[p.color]++ ? 0.045 : -0.045), by, bz + 0.01 + 0.05 + 0.002])
    })
  },
}
