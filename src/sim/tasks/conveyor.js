// Conveyor package handling, in the spirit of the humanoid logistics demos: packages arrive on the input belt
// (robot's left) in a random orientation. The operator turns each one shipping-tag up and sets it on the output
// belt (robot's right), which carries it away. One episode = a fixed number of packages.
import { LAYOUT } from './conveyor.layout.js'

const { pool: POOL, beltX: BELT_X, beltTop: BELT_TOP, rollerRadius: ROLLER_R, spawnY: SPAWN_Y, exitY: EXIT_Y, outputStartY: OUTPUT_START_Y, sizes: SIZES } = LAYOUT
const PACKAGES_PER_EPISODE = 5
const SPAWN_INTERVAL = 4              // s between packages, once the belt in front of the spawn point is clear
const TAG_UP = Math.cos(20 * Math.PI / 180)
const PARK = i => [-3 - 0.3 * i, 0, 0.05]

// Quaternions (w, x, y, z) putting the body's +z (tag) on each world face
const FACE_UP = [
  [1, 0, 0, 0],                       // tag up
  [0, 1, 0, 0],                       // tag down
  [0.7071068, 0.7071068, 0, 0],       // tag toward -y / +y
  [0.7071068, -0.7071068, 0, 0],
  [0.7071068, 0, 0.7071068, 0],       // tag toward +x / -x
  [0.7071068, 0, -0.7071068, 0],
]

function mulQuat([aw, ax, ay, az], [bw, bx, by, bz]) {
  return [
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
  ]
}

// Height of the body centre above the belt for a given orientation (box resting on whichever face is down)
function restHeight(half, face) {
  const h = face < 2 ? half[2] : face < 4 ? half[1] : half[0]
  return BELT_TOP + h + 0.002
}

const round = (x, p = 3) => Number(x.toFixed(p))

export default {
  name: 'conveyor',
  instruction: 'Take each package from the left belt, turn it so the shipping tag faces up, and put it on the right belt',
  title: 'Tag up, onto the right belt',
  scene: 'mujoco/conveyor.xml',
  objects: Array.from({ length: POOL }, (_, i) => `package${i}`),
  timeout: 150,
  materials: {
    cardboard: { roughness: 0.95 }, tag: { roughness: 0.6 }, tagbar: { roughness: 0.6 },
    roller: { roughness: 0.4, metalness: 0.6 }, rail: { roughness: 0.45, metalness: 0.7 }, leg: { roughness: 0.6, metalness: 0.5 },
  },

  randomize(rng) {
    const u = (lo, hi) => lo + (hi - lo) * rng()
    const params = {}
    for (let i = 0; i < POOL; i++) params[`package${i}`] = { mass: u(0.2, 0.8), friction: u(0.5, 0.9) }
    return params
  },

  reset(sim, rng) {
    const u = (lo, hi) => lo + (hi - lo) * rng()
    for (let i = 0; i < POOL; i++) sim.placeObject(i, PARK(i))
    const beltSpeed = u(0.05, 0.09)                     // m/s
    // belt motors: rollers spin about +x; positive carries toward -y (from the robot's left to its right)
    const { m, d } = sim
    for (let a = 0; a < m.nu; a++) {
      if (sim.name('mjOBJ_ACTUATOR', a).startsWith('belt_')) d.ctrl[a] = Math.fround(beltSpeed / ROLLER_R)
    }
    // Spawn plan: distinct pool bodies in random order (so the size mix varies between episodes), which face
    // carries the tag, yaw
    const bodies = Array.from({ length: POOL }, (_, i) => i)
    for (let i = bodies.length - 1; i > 0; i--) {
      const j = Math.floor(rng() * (i + 1));
      [bodies[i], bodies[j]] = [bodies[j], bodies[i]]
    }
    const order = bodies.slice(0, PACKAGES_PER_EPISODE).map(body => ({ body, face: Math.floor(rng() * 6), yaw: u(-0.4, 0.4) }))
    sim.taskState = { order, spawned: 0, active: [], delivered: [], nextSpawnAt: 0.5 }
    return { beltSpeed, packages: order }
  },

  update(sim) {
    const st = sim.taskState
    const t = sim.d.time - sim.startTime

    // Spawn the next package once the pool body is free and the spawn point is clear
    if (st.spawned < st.order.length && t >= st.nextSpawnAt) {
      const plan = st.order[st.spawned] // pool bodies in the plan are distinct, so this one is free
      const spawnClear = !st.active.some(a => a.onInput && sim.objectPos(a.body)[1] > SPAWN_Y - 0.25)
      if (spawnClear) {
        const half = SIZES[Math.floor(plan.body / 2)]
        const quat = mulQuat([Math.cos(plan.yaw / 2), 0, 0, Math.sin(plan.yaw / 2)], FACE_UP[plan.face])
        sim.teleportObject(plan.body, [BELT_X, SPAWN_Y, restHeight(half, plan.face)], quat)
        st.active.push({ body: plan.body, index: st.spawned, onInput: true })
        st.spawned++
        st.nextSpawnAt = t + SPAWN_INTERVAL
      }
    }

    // Track packages leaving the output belt or falling
    for (const a of st.active.slice()) {
      const [, y, z] = sim.objectPos(a.body)
      if (y < OUTPUT_START_Y) a.onInput = false
      let outcome = null
      if (z < 0.5) outcome = 'dropped'
      else if (y < EXIT_Y) outcome = sim.objectUp(a.body) > TAG_UP ? 'correct' : 'wrong_face'
      if (outcome) {
        st.delivered.push({ index: a.index, outcome, time: round(t, 2) })
        st.active.splice(st.active.indexOf(a), 1)
        sim.teleportObject(a.body, PARK(a.body))
      }
    }

    if (st.delivered.length === st.order.length) {
      return st.delivered.every(r => r.outcome === 'correct') ? 'success' : 'partial'
    }
    return null
  },

  // Pick zone at the end stop of the input belt (left hand) and the start of the output belt (right hand);
  // the other hand cannot cross that far at belt height, so a package changes hands in the middle
  reachTargets() {
    return [
      { side: 0, point: [BELT_X, 0.17, BELT_TOP + 0.08] },
      { side: 1, point: [BELT_X, OUTPUT_START_Y - 0.08, BELT_TOP + 0.1] },
    ]
  },

  hud(sim) {
    const st = sim.taskState
    const ok = st.delivered.filter(r => r.outcome === 'correct').length
    return `${st.delivered.length}/${st.order.length} done, ${ok} tag-up`
  },

  result(sim) {
    const st = sim.taskState
    return { packages: st.order.length, delivered: st.delivered, correct: st.delivered.filter(r => r.outcome === 'correct').length }
  },

  // Headless check: packages already turned tag-up on the output belt; the belt must carry them off and score them
  solved(sim) {
    const st = sim.taskState
    st.active = []
    st.order.forEach((plan, n) => {
      const half = SIZES[Math.floor(plan.body / 2)]
      sim.teleportObject(plan.body, [BELT_X, OUTPUT_START_Y - 0.08 - 0.14 * n, restHeight(half, 0)])
      st.active.push({ body: plan.body, index: n, onInput: false })
    })
    st.spawned = st.order.length
  },
}
