// Cube sorting: six printed PLA cubes, two of each colour, into the matching bins.

const CUBES = ['red', 'red', 'green', 'green', 'blue', 'blue']
const CUBE_HALF = 0.025
const CUBE_Z = 0.79 + CUBE_HALF + 0.001
// Spawn slots on the table in front of the robot, clear of the bins (the green bin's wall starts at x=0.34; a
// yawed cube reaches 3.5 cm from its centre): a 2 x 3 grid 9 x 12 cm apart with +-8 mm jitter keeps centres
// > 7.4 cm apart, more than two half-diagonals (7.1 cm), so neighbours never overlap
const SLOT_X = [0.20, 0.29]
const SLOT_Y = [-0.12, 0, 0.12]
const JITTER = 0.008
const YAW_JITTER = 0.8
const BIN_INNER = [0.09, 0.07]   // bin interior half extents (0.108 x 0.088) minus a margin
const BIN_MAX_Z = 0.86           // cube centre must be below the bin rim (0.87)

/** The six slots in random order, each jittered. */
function samplePositions(rng) {
  const slots = SLOT_X.flatMap(x => SLOT_Y.map(y => [x, y]))
  for (let i = slots.length - 1; i > 0; i--) {
    const j = Math.floor(rng() * (i + 1));
    [slots[i], slots[j]] = [slots[j], slots[i]]
  }
  return slots.map(([x, y]) => [x + (rng() * 2 - 1) * JITTER, y + (rng() * 2 - 1) * JITTER])
}

export default {
  name: 'cube_sort',
  instruction: 'Sort the cubes into the bins of the same colour',
  title: 'Cubes into matching bins',
  scene: 'mujoco/cube_sort.xml',
  objects: CUBES.map((_, i) => `cube${i}`),
  dropZ: 0.68,
  timeout: 120,
  materials: Object.fromEntries([...['red', 'green', 'blue'].map(c => [c, { roughness: 0.55 }]), ...['red_bin', 'green_bin', 'blue_bin'].map(c => [c, { roughness: 0.7 }])]), // printed PLA, plastic bins

  randomize(rng) {
    const u = (lo, hi) => lo + (hi - lo) * rng()
    return Object.fromEntries(CUBES.map((_, i) => [`cube${i}`, { mass: u(0.04, 0.09), friction: u(0.4, 0.6) }]))
  },

  reset(sim, rng) {
    const positions = samplePositions(rng)
    const cubes = CUBES.map((color, i) => {
      const pos = [positions[i][0], positions[i][1], CUBE_Z]
      const yaw = (rng() * 2 - 1) * YAW_JITTER
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

  // Where a hand must be able to let go for the goal to be achievable: above each bin's near half
  reachTargets(sim) {
    const { bins } = sim.layout
    return [
      { side: 0, point: [bins.red[0], bins.red[1], 0.95] },
      { side: 1, point: [bins.blue[0], bins.blue[1], 0.95] },
      { side: 0, point: [bins.green[0] - 0.04, bins.green[1] + 0.03, 0.95] },
      { side: 1, point: [bins.green[0] - 0.04, bins.green[1] - 0.03, 0.95] },
    ]
  },

  // Headless check: cubes placed in their bins
  solved(sim) {
    const count = { red: 0, green: 0, blue: 0 }
    CUBES.forEach((color, i) => {
      const [bx, by, bz] = sim.layout.bins[color]
      sim.teleportObject(i, [bx + (count[color]++ ? 0.04 : -0.04), by, bz + 0.002 + CUBE_HALF + 0.002])
    })
  },
}
