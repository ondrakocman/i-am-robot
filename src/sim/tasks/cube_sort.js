// Cube sorting: six printed PLA cubes, two of each colour, into the matching bins.

const CUBES = ['red', 'red', 'green', 'green', 'blue', 'blue']
const SLOTS = [[0.26, 0.06], [0.33, 0.06], [0.26, -0.06], [0.33, -0.06], [0.295, 0.14], [0.295, -0.14]]
const CUBE_Z = 0.816
const JITTER = 0.012
const BIN_INNER = [0.09, 0.07]   // bin interior half extents minus a margin (bin is 0.22 x 0.18 outside)
const BIN_MAX_Z = 0.86           // cube centre must be below the bin rim (0.87)

function shuffle(arr, rng) {
  for (let i = arr.length - 1; i > 0; i--) {
    const j = Math.floor(rng() * (i + 1));
    [arr[i], arr[j]] = [arr[j], arr[i]]
  }
  return arr
}

export default {
  name: 'cube_sort',
  instruction: 'Sort the cubes into the bins of the same colour',
  title: 'Cubes into matching bins',
  scene: 'mujoco/cube_sort.xml',
  objects: CUBES.map((_, i) => `cube${i}`),
  dropZ: 0.68,
  timeout: 120,

  randomize(rng) {
    const u = (lo, hi) => lo + (hi - lo) * rng()
    return Object.fromEntries(CUBES.map((_, i) => [`cube${i}`, { mass: u(0.04, 0.09), friction: u(0.4, 0.6) }]))
  },

  reset(sim, rng) {
    const slots = shuffle(SLOTS.slice(), rng)
    const cubes = CUBES.map((color, i) => {
      const [sx, sy] = slots[i]
      const pos = [sx + (rng() * 2 - 1) * JITTER, sy + (rng() * 2 - 1) * JITTER, CUBE_Z]
      const yaw = (rng() * 2 - 1) * 0.8
      sim.placeObject(i, pos, yaw)
      return { color, pos, yaw }
    })
    const bins = {}
    for (const color of ['red', 'green', 'blue']) {
      const b = 3 * sim.bodyId(`bin_${color}`)
      bins[color] = Array.from(sim.m.body_pos.slice(b, b + 3))
    }
    return { cubes, bins }
  },

  goal(sim) {
    return CUBES.every((color, i) => {
      const [x, y, z] = sim.objectPos(i)
      const [bx, by] = sim.layout.bins[color]
      return Math.abs(x - bx) < BIN_INNER[0] && Math.abs(y - by) < BIN_INNER[1] && z < BIN_MAX_Z
    })
  },

  // Headless check: cubes placed in their bins
  solved(sim) {
    const count = { red: 0, green: 0, blue: 0 }
    CUBES.forEach((color, i) => {
      const [bx, by, bz] = sim.layout.bins[color]
      sim.teleportObject(i, [bx + (count[color]++ ? 0.04 : -0.04), by, bz + 0.002 + 0.025 + 0.002])
    })
  },
}
