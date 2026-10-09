// Conveyor package handling after Figure's 24-hour logistics demo: packages slide down a chute on the robot's
// left onto a flat work plate in front of it. The operator turns each one shipping-label up and sets it on the
// output belt (robot's right), which carries it away. One episode = a fixed number of packages.
import { LAYOUT } from './conveyor.layout.js'

const { pool: POOL, beltX: BELT_X, beltTop: BELT_TOP, rollerRadius: ROLLER_R, spawn: SPAWN, chuteNormal: CHUTE_N, plateY: PLATE_Y, exitY: EXIT_Y, outputStartY: OUTPUT_START_Y, sizes: SIZES } = LAYOUT
const PACKAGES_PER_EPISODE = 5
const SPAWN_INTERVAL = [5, 9]         // s between packages, once the top of the chute is clear
const SPAWN_CLEARANCE = 0.25          // m: no other package this close to the spawn point
const TAG_UP = Math.cos(20 * Math.PI / 180)
const PARK = i => [-3 - 0.3 * i, 0, 0.05]

// Quaternions (w, x, y, z) putting the body's +z (label) on each world face
const FACE_UP = [
  [1, 0, 0, 0],                       // label up
  [0, 1, 0, 0],                       // label down
  [0.7071068, 0.7071068, 0, 0],       // label toward -y / +y
  [0.7071068, -0.7071068, 0, 0],
  [0.7071068, 0, 0.7071068, 0],       // label toward +x / -x
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

// Rotates v by the unit quaternion q (w, x, y, z)
function rotate([w, x, y, z], [vx, vy, vz]) {
  const tx = 2 * (y * vz - z * vy), ty = 2 * (z * vx - x * vz), tz = 2 * (x * vy - y * vx)
  return [vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx)]
}

/**
 * Where a package with this plan (pool body, label face, yaw) appears on the chute: resting on the spawn
 * point of the chute surface, lifted along the surface normal by the rotated box's extent in that direction
 * plus a small gap, so no face starts inside the sheet whatever the orientation.
 */
function spawnPose(plan) {
  const half = SIZES[Math.floor(plan.body / 2)]
  const quat = mulQuat([Math.cos(plan.yaw / 2), 0, 0, Math.sin(plan.yaw / 2)], FACE_UP[plan.face])
  let extent = 0
  for (let i = 0; i < 3; i++) {
    const axis = rotate(quat, [i === 0 ? 1 : 0, i === 1 ? 1 : 0, i === 2 ? 1 : 0])
    extent += Math.abs(axis[0] * CHUTE_N[0] + axis[1] * CHUTE_N[1] + axis[2] * CHUTE_N[2]) * half[i]
  }
  const lift = extent + 0.004
  return { pos: [SPAWN[0] + CHUTE_N[0] * lift, SPAWN[1] + CHUTE_N[1] * lift, SPAWN[2] + CHUTE_N[2] * lift], quat }
}

const round = (x, p = 3) => Number(x.toFixed(p))

export default {
  name: 'conveyor',
  instruction: 'Take each package from the chute, turn it so the shipping label faces up, and put it on the right belt',
  title: 'Label up, onto the right belt',
  scene: 'mujoco/conveyor.xml',
  objects: Array.from({ length: POOL }, (_, i) => `package${i}`),
  timeout: 150,
  materials: {
    cardboard: { roughness: 0.95 },
    plate: { roughness: 0.35, metalness: 0.8 }, chute: { roughness: 0.4, metalness: 0.75 },
    roller: { roughness: 0.4, metalness: 0.6 }, rail: { roughness: 0.45, metalness: 0.7 }, leg: { roughness: 0.6, metalness: 0.5 },
  },
  // each package's label is a printed shipping label (address, barcode, QR code), different per package
  geometry: Object.fromEntries(Array.from({ length: POOL }, (_, i) => [`label${i}`, { shippingLabel: { seed: i } }])),

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
    // belt motors: rollers spin about +x; positive carries toward -y (away from the robot's left)
    const { m, d } = sim
    for (let a = 0; a < m.nu; a++) {
      if (sim.name('mjOBJ_ACTUATOR', a).startsWith('belt_')) d.ctrl[a] = Math.fround(beltSpeed / ROLLER_R)
    }
    // Spawn plan: distinct pool bodies in random order (so the size mix varies between episodes), which face
    // carries the label, yaw, and the pause before each one is released onto the chute
    const bodies = Array.from({ length: POOL }, (_, i) => i)
    for (let i = bodies.length - 1; i > 0; i--) {
      const j = Math.floor(rng() * (i + 1));
      [bodies[i], bodies[j]] = [bodies[j], bodies[i]]
    }
    const order = bodies.slice(0, PACKAGES_PER_EPISODE).map(body => ({ body, face: Math.floor(rng() * 6), yaw: u(-0.5, 0.5), after: round(u(...SPAWN_INTERVAL), 2) }))
    sim.taskState = { order, spawned: 0, active: [], delivered: [], nextSpawnAt: 1 }
    return { beltSpeed, packages: order }
  },

  update(sim) {
    const st = sim.taskState
    const t = sim.d.time - sim.startTime

    // Release the next package at the top of the chute once it is due and the spawn point is clear
    if (st.spawned < st.order.length && t >= st.nextSpawnAt) {
      const plan = st.order[st.spawned] // pool bodies in the plan are distinct, so this one is free
      const clear = !st.active.some(a => {
        const [x, y, z] = sim.objectPos(a.body)
        return Math.hypot(x - SPAWN[0], y - SPAWN[1], z - SPAWN[2]) < SPAWN_CLEARANCE
      })
      if (clear) {
        const { pos, quat } = spawnPose(plan)
        sim.teleportObject(plan.body, pos, quat)
        st.active.push({ body: plan.body, index: st.spawned })
        st.spawned++
        st.nextSpawnAt = t + plan.after
      }
    }

    // Track packages leaving the output belt or falling
    for (const a of st.active.slice()) {
      const [, y, z] = sim.objectPos(a.body)
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

  // Pick zone where packages come to rest at the foot of the chute (left hand) and the start of the output
  // belt (right hand); the other hand cannot cross that far, so a package changes hands over the plate
  reachTargets() {
    return [
      { side: 0, point: [SPAWN[0], PLATE_Y[1] - 0.12, BELT_TOP + 0.08] },
      { side: 1, point: [BELT_X, OUTPUT_START_Y - 0.08, BELT_TOP + 0.1] },
    ]
  },

  hud(sim) {
    const st = sim.taskState
    const ok = st.delivered.filter(r => r.outcome === 'correct').length
    return `${st.delivered.length}/${st.order.length} done, ${ok} label-up`
  },

  result(sim) {
    const st = sim.taskState
    return { packages: st.order.length, delivered: st.delivered, correct: st.delivered.filter(r => r.outcome === 'correct').length }
  },

  // Headless check: every spawn pose the plan can draw (each pool body, face and yaw extreme) must be a valid,
  // non-penetrating state
  spawnPoses() {
    const poses = []
    for (let body = 0; body < POOL; body++) {
      for (let face = 0; face < 6; face++) {
        for (const yaw of [-0.5, 0, 0.5]) poses.push({ body, ...spawnPose({ body, face, yaw }) })
      }
    }
    return poses
  },

  // Headless check of the scoring: packages label-down on the output belt must end the episode as 'partial'
  // with every package scored wrong_face
  failureCase(sim) {
    const st = sim.taskState
    st.active = []
    st.order.forEach((plan, n) => {
      const half = SIZES[Math.floor(plan.body / 2)]
      sim.teleportObject(plan.body, [BELT_X, OUTPUT_START_Y - 0.08 - 0.14 * n, BELT_TOP + half[2] + 0.002], FACE_UP[1])
      st.active.push({ body: plan.body, index: n })
    })
    st.spawned = st.order.length
    return { outcome: 'partial', result: r => r.delivered.every(d => d.outcome === 'wrong_face') }
  },

  // Headless check: packages already turned label-up on the output belt; the belt must carry them off and score them
  solved(sim) {
    const st = sim.taskState
    st.active = []
    st.order.forEach((plan, n) => {
      const half = SIZES[Math.floor(plan.body / 2)]
      sim.teleportObject(plan.body, [BELT_X, OUTPUT_START_Y - 0.08 - 0.14 * n, BELT_TOP + half[2] + 0.002])
      st.active.push({ body: plan.body, index: n })
    })
    st.spawned = st.order.length
  },
}
