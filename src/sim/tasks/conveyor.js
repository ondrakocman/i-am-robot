// Conveyor package handling, in the spirit of the humanoid logistics demos: packages arrive on the input belt
// (robot's left) in a random orientation. The operator turns each one shipping-tag up and sets it on the output
// belt (robot's right), which carries it away. One episode = a fixed number of packages.

const POOL = 6                        // package bodies in the scene (3 sizes x 2)
const PACKAGES_PER_EPISODE = 5
const ROLLER_R = 0.014
const BELT_X = 0.305
const BELT_TOP = 0.79
const SPAWN_Y = 0.83                  // far end of the input belt
const EXIT_Y = -0.84                  // far end of the output belt: packages past this are delivered
const PICK_ZONE_Y = 0.3               // input belt is "busy" while a package is still above this
const OUTPUT_START_Y = -0.12
const TAG_UP = Math.cos(20 * Math.PI / 180)
const PARK = i => [-3 - 0.3 * i, 0, 0.05]
const SIZES = [[0.06, 0.04, 0.03], [0.075, 0.05, 0.025], [0.04, 0.04, 0.04]]

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

export default {
  name: 'conveyor',
  instruction: 'Take each package from the left belt, turn it so the shipping tag faces up, and put it on the right belt',
  title: 'Tag up, onto the right belt',
  scene: 'mujoco/conveyor.xml',
  objects: Array.from({ length: POOL }, (_, i) => `package${i}`),
  sceneActuators: 50,
  timeout: 150,

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
    const w = beltSpeed / ROLLER_R
    // belt motors: rollers spin about +x; +w carries toward -y (from the robot's left to its right)
    const { m, d } = sim
    for (let a = 0; a < m.nu; a++) {
      const name = sim.name('mjOBJ_ACTUATOR', a)
      if (name.startsWith('belt_')) d.ctrl[a] = Math.fround(w) // float32 so the log replays exactly
    }
    // spawn plan: which pool body, which face up, what yaw, in order
    const order = []
    for (let n = 0; n < PACKAGES_PER_EPISODE; n++) {
      order.push({ body: n % POOL, face: Math.floor(rng() * 6), yaw: u(-0.4, 0.4) })
    }
    this.state = { order, spawned: 0, active: [], delivered: [], beltSpeed, nextSpawnAt: 0.5 }
    return { beltSpeed, packages: order }
  },

  update(sim) {
    const st = this.state
    const t = sim.d.time - sim.startTime

    // Spawn the next package once the previous one has moved down the belt and the pool body is free
    if (st.spawned < st.order.length && t >= st.nextSpawnAt) {
      const plan = st.order[st.spawned]
      const inUse = st.active.some(a => a.body === plan.body)
      const beltBusy = st.active.some(a => a.onInput && sim.objectPos(a.body)[1] > SPAWN_Y - 0.25)
      if (!inUse && !beltBusy) {
        const half = SIZES[Math.floor(plan.body / 2)]
        const quat = mulQuat([Math.cos(plan.yaw / 2), 0, 0, Math.sin(plan.yaw / 2)], FACE_UP[plan.face])
        sim.teleportObject(plan.body, [BELT_X, SPAWN_Y, restHeight(half, plan.face)], quat)
        st.active.push({ body: plan.body, index: st.spawned, onInput: true })
        st.spawned++
        st.nextSpawnAt = t + 4
      }
    }

    // Track packages leaving the output belt or falling
    for (const a of st.active.slice()) {
      const [x, y, z] = sim.objectPos(a.body)
      if (y < OUTPUT_START_Y) a.onInput = false
      let outcome = null
      if (z < 0.5) outcome = 'dropped'
      else if (y < EXIT_Y) outcome = sim.objectUp(a.body) > TAG_UP ? 'correct' : 'wrong_face'
      if (outcome) {
        st.delivered.push({ index: a.index, outcome, time: t })
        st.active.splice(st.active.indexOf(a), 1)
        sim.teleportObject(a.body, PARK(a.body))
      }
    }

    if (st.delivered.length === st.order.length) {
      return st.delivered.every(r => r.outcome === 'correct') ? 'success' : 'partial'
    }
    return null
  },

  hud(sim) {
    const st = this.state
    const ok = st.delivered.filter(r => r.outcome === 'correct').length
    return `${st.delivered.length}/${st.order.length} done, ${ok} tag-up`
  },

  result(sim) {
    const st = this.state
    return { packages: st.order.length, delivered: st.delivered, correct: st.delivered.filter(r => r.outcome === 'correct').length }
  },

  // Headless check: packages already turned tag-up on the output belt; the belt must carry them off and score them
  solved(sim) {
    const st = this.state
    st.active = []
    st.order.forEach((plan, n) => {
      const half = SIZES[Math.floor(plan.body / 2)]
      sim.teleportObject(plan.body, [BELT_X, OUTPUT_START_Y - 0.08 - 0.14 * n, restHeight(half, 0)])
      st.active.push({ body: plan.body, index: n, onInput: false })
    })
    st.spawned = st.order.length
  },
}
