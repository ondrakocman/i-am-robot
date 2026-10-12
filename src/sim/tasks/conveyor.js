// Conveyor package handling after Figure's 24-hour logistics demo: packages slide down a chute on the robot's
// left onto a flat work plate in front of it. The operator turns each one shipping-label up and sets it on the
// output belt (robot's right), which carries it away. One episode = a fixed number of packages drawn from a
// pool of rigid cardboard boxes (MuJoCo bodies) and soft poly-mailer bags (XPBD, see soft.js).
import { LAYOUT } from './conveyor.layout.js'
import { sampleKeys } from '../autopilot.js'

const { pool: POOL, beltX: BELT_X, beltTop: BELT_TOP, rollerRadius: ROLLER_R, spawn: SPAWN, chuteNormal: CHUTE_N, plateX: PLATE_X, plateY: PLATE_Y, exitY: EXIT_Y, outputStartY: OUTPUT_START_Y, sizes: SIZES } = LAYOUT
const PACKAGES_PER_EPISODE = 5
const SPAWN_INTERVAL = [5, 9]         // s between packages, once the top of the chute is clear
const SPAWN_CLEARANCE = 0.25          // m: no other package this close to the spawn point
const TAG_UP = Math.cos(20 * Math.PI / 180)
// Soft parcels: half extents of the undeformed bag, lattice cells, mass, label
const BAGS = {
  bag0: { half: [0.08, 0.06, 0.022], cells: [6, 4, 2], mass: 0.3, label: { seed: 6 } },
  bag1: { half: [0.07, 0.05, 0.025], cells: [5, 4, 2], mass: 0.25, label: { seed: 7 } },
}
// The package pool: rigid boxes (two of each size) and the bags
const ITEMS = [
  ...Array.from({ length: POOL }, (_, i) => ({ kind: 'rigid', id: i, body: i, half: SIZES[Math.floor(i / 2)] })),
  ...Object.entries(BAGS).map(([name, def], k) => ({ kind: 'soft', id: POOL + k, name, half: def.half })),
]
const PARK = id => [-3 - 0.3 * id, 0, 0.05]

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
 * Where a package (half extents, label face, yaw) appears on the chute: resting on the spawn point of the chute
 * surface, lifted along the surface normal by the rotated box's extent in that direction plus a small gap, so
 * no face starts inside the sheet whatever the orientation.
 */
function spawnPose(half, face, yaw) {
  const quat = mulQuat([Math.cos(yaw / 2), 0, 0, Math.sin(yaw / 2)], FACE_UP[face])
  let extent = 0
  for (let i = 0; i < 3; i++) {
    const axis = rotate(quat, [i === 0 ? 1 : 0, i === 1 ? 1 : 0, i === 2 ? 1 : 0])
    extent += Math.abs(axis[0] * CHUTE_N[0] + axis[1] * CHUTE_N[1] + axis[2] * CHUTE_N[2]) * half[i]
  }
  const lift = extent + 0.004
  return { pos: [SPAWN[0] + CHUTE_N[0] * lift, SPAWN[1] + CHUTE_N[1] * lift, SPAWN[2] + CHUTE_N[2] * lift], quat }
}

// Rigid or soft, the task reads and moves a package the same way
const itemPos = (sim, it) => it.kind === 'soft' ? sim.softPos(it.name) : sim.objectPos(it.body)
const itemUp = (sim, it) => it.kind === 'soft' ? sim.softUp(it.name) : sim.objectUp(it.body)
const itemTeleport = (sim, it, pos, quat) => it.kind === 'soft' ? sim.teleportSoft(it.name, pos, quat, true) : sim.teleportObject(it.body, pos, quat)
const itemPark = (sim, it) => it.kind === 'soft' ? sim.teleportSoft(it.name, PARK(it.id), [1, 0, 0, 0], false) : sim.teleportObject(it.body, PARK(it.id))
const onBelt = (it, n) => [BELT_X, OUTPUT_START_Y - 0.08 - 0.14 * n, BELT_TOP + it.half[2] + (it.kind === 'soft' ? 0.004 : 0.002)]

const round = (x, p = 3) => Number(x.toFixed(p))

// Two-sided sanity of a bag at the end of a contact case: on the plate, intact, not thrown
function bagSane(sim, name) {
  const bag = sim.soft.byName[name]
  if (!bag.finite()) return `${name} went non-finite`
  const [x, y, z] = bag.centroid()
  if (x < PLATE_X[0] || x > PLATE_X[1] || y < PLATE_Y[0] || y > PLATE_Y[1] + 0.1 || z < BELT_TOP || z > BELT_TOP + 0.2) return `${name} is not on the plate or the chute foot (${[x, y, z].map(v => v.toFixed(2))})`
  const { volume, inverted } = bag.volumeState()
  if (inverted > 2 || volume < 0.9) return `${name} is crushed (${inverted} inverted tets, ${(volume * 100).toFixed(0)}% of rest volume)`
  if (sim.caseStats?.peakSpeed > 2) return `${name} was thrown (peak particle speed ${sim.caseStats.peakSpeed.toFixed(1)} m/s)`
  return null
}

// Parks every spawned package and cancels the rest of the plan (headless contact cases)
function clearPlan(sim) {
  const st = sim.taskState
  for (const a of st.active) itemPark(sim, ITEMS[a.item])
  st.active = []
  st.spawned = st.order.length
}

export default {
  name: 'conveyor',
  instruction: 'Take each package from the chute, turn it so the shipping label faces up, and put it on the right belt',
  title: 'Label up, onto the right belt',
  scene: 'mujoco/conveyor.xml',
  objects: Array.from({ length: POOL }, (_, i) => `package${i}`),
  soft: { bodies: BAGS, belts: ['belt_out_roller'] },
  timesteps: [0.002], // the soft coupling (soft.js) is tuned for this step
  timeout: 150,
  materials: {
    cardboard: { roughness: 0.95 },
    plate: { roughness: 0.35, metalness: 0.8 }, chute: { roughness: 0.4, metalness: 0.75 },
    roller: { roughness: 0.4, metalness: 0.6 }, rail: { roughness: 0.45, metalness: 0.7 }, leg: { roughness: 0.6, metalness: 0.5 },
  },
  // each box's label is a printed shipping label (address, barcode, QR code), different per package
  geometry: Object.fromEntries(Array.from({ length: POOL }, (_, i) => [`label${i}`, { shippingLabel: { seed: i } }])),

  randomize(rng) {
    const u = (lo, hi) => lo + (hi - lo) * rng()
    const params = {}
    for (let i = 0; i < POOL; i++) params[`package${i}`] = { mass: u(0.2, 0.8), friction: u(0.5, 0.9) }
    return params
  },

  // bags: contents weight, film friction, and how squishy (edge compliance, m/N)
  randomizeSoft(rng) {
    const u = (lo, hi) => lo + (hi - lo) * rng()
    const params = {}
    for (const name of Object.keys(BAGS)) params[name] = { mass: u(0.15, 0.4), friction: u(0.5, 0.9), edgeCompliance: u(1e-3, 4e-3) }
    return params
  },

  reset(sim, rng) {
    const u = (lo, hi) => lo + (hi - lo) * rng()
    for (const it of ITEMS) {
      if (it.kind === 'soft') sim.placeSoft(it.name, PARK(it.id), [1, 0, 0, 0], false)
      else sim.placeObject(it.body, PARK(it.id))
    }
    const beltSpeed = u(0.05, 0.09)                     // m/s
    // belt motors: rollers spin about +x; positive carries toward -y (away from the robot's left)
    const { m, d } = sim
    for (let a = 0; a < m.nu; a++) {
      if (sim.name('mjOBJ_ACTUATOR', a).startsWith('belt_')) d.ctrl[a] = Math.fround(beltSpeed / ROLLER_R)
    }
    // Spawn plan: distinct pool items in random order (so the mix of boxes and bags varies between episodes),
    // which face carries the label, yaw, and the pause before each one is released onto the chute
    const items = ITEMS.map((_, i) => i)
    for (let i = items.length - 1; i > 0; i--) {
      const j = Math.floor(rng() * (i + 1));
      [items[i], items[j]] = [items[j], items[i]]
    }
    // Boxes arrive on any face; bags arrive label-up: this hand cannot pinch a flat bag up to turn it over
    // (see the contact cases), so a label-down bag could never be delivered correctly
    const order = items.slice(0, PACKAGES_PER_EPISODE).map(item => ({ item, face: ITEMS[item].kind === 'soft' ? 0 : Math.floor(rng() * 6), yaw: u(-0.5, 0.5), after: round(u(...SPAWN_INTERVAL), 2) }))
    sim.taskState = { order, spawned: 0, active: [], delivered: [], nextSpawnAt: 1 }
    return {
      beltSpeed,
      packages: order.map(p => ({ kind: ITEMS[p.item].kind, id: ITEMS[p.item].id, face: p.face, yaw: p.yaw, after: p.after })),
      labels: Object.fromEntries(ITEMS.map(it => [it.kind === 'soft' ? it.name : `package${it.body}`, it.kind === 'soft' ? BAGS[it.name].label.seed : it.body])),
    }
  },

  update(sim) {
    const st = sim.taskState
    const t = sim.d.time - sim.startTime

    // Release the next package at the top of the chute once it is due and the spawn point is clear
    if (st.spawned < st.order.length && t >= st.nextSpawnAt) {
      const plan = st.order[st.spawned] // pool items in the plan are distinct, so this one is free
      const clear = !st.active.some(a => {
        const [x, y, z] = itemPos(sim, ITEMS[a.item])
        return Math.hypot(x - SPAWN[0], y - SPAWN[1], z - SPAWN[2]) < SPAWN_CLEARANCE
      })
      if (clear) {
        const it = ITEMS[plan.item]
        const { pos, quat } = spawnPose(it.half, plan.face, plan.yaw)
        itemTeleport(sim, it, pos, quat)
        st.active.push({ item: plan.item, index: st.spawned })
        st.spawned++
        st.nextSpawnAt = t + plan.after
      }
    }

    // Track packages leaving the output belt or falling
    for (const a of st.active.slice()) {
      const it = ITEMS[a.item]
      const [, y, z] = itemPos(sim, it)
      let outcome = null
      if (z < 0.5) outcome = 'dropped'
      else if (y < EXIT_Y) outcome = itemUp(sim, it) > TAG_UP ? 'correct' : 'wrong_face'
      if (outcome) {
        st.delivered.push({ index: a.index, outcome, time: round(t, 2) })
        st.active.splice(st.active.indexOf(a), 1)
        itemPark(sim, it)
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
      { side: 0, point: [SPAWN[0], PLATE_Y[1] - 0.08, BELT_TOP + 0.08] }, // where boxes stop (bags stop ~8 cm further, at the chute foot)
      { side: 0, point: [SPAWN[0], PLATE_Y[1], BELT_TOP + 0.08] },
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

  // Headless check: every spawn pose the plan can draw (each pool item, face and yaw extreme) must be a valid,
  // non-penetrating state
  spawnPoses() {
    const poses = []
    for (const it of ITEMS) {
      for (let face = 0; face < (it.kind === 'soft' ? 1 : 6); face++) {
        for (const yaw of [-0.5, 0, 0.5]) {
          const { pos, quat } = spawnPose(it.half, face, yaw)
          poses.push(it.kind === 'soft' ? { soft: it.name, pos, quat } : { body: it.body, pos, quat })
        }
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
      const it = ITEMS[plan.item]
      itemTeleport(sim, it, onBelt(it, n), FACE_UP[1])
      st.active.push({ item: plan.item, index: n })
    })
    st.spawned = st.order.length
    return { outcome: 'partial', result: r => r.delivered.every(d => d.outcome === 'wrong_face') }
  },

  // Headless contact cases (scripted, recorded and replayed by sim-check): the soft parcels meeting a hand, a
  // box and each other. Each clears the spawn plan first so nothing slides into the test.
  contactCases: [
    {
      // the operator's most common bag contacts: pressing on it (the hand must come away clean, the bag must
      // spring back) and pushing it along the plate from the side (friction against the plate). Pressing and
      // dragging does not move a bag whose skin friction is the same on both sides: the plate carries the
      // extra weight, so the hand slides over the bag instead
      name: 'left hand presses into a bag, lifts off clean, then pushes it along the plate',
      duration: 14,
      setup(sim) {
        clearPlan(sim)
        sim.teleportSoft('bag0', [0.3, 0.2, BELT_TOP + BAGS.bag0.half[2] + 0.004], [1, 0, 0, 0], true)
        sim.caseStats = { maxZ: 0 }
      },
      hands(sim, t) {
        const r = sim.readyGrip[0], cx = 0.3, cy = 0.2, top = BELT_TOP + 2 * BAGS.bag0.half[2], mid = BELT_TOP + BAGS.bag0.half[2] // push at the bag's centre height: higher tips it
        const keys = [
          [0, ...r, 0, 0], [2.5, ...r, 0, 0], [4, cx, cy, top + 0.12, 0, 0], [5.5, cx, cy, top - 0.03, 0, 0], [6.5, cx, cy, top - 0.03, 0, 0],
          [7.5, cx, cy, top + 0.12, 0, 0], [9, cx - 0.16, cy, mid + 0.08, 0, 0], [10, cx - 0.16, cy, mid, 0, 0], [12, cx - 0.04, cy, mid, 0, 0],
          [13, cx - 0.04, cy, mid + 0.12, 0, 0], [15, ...r, 0, 0],
        ]
        if (sim.caseStats && t > 7 && t < 9) sim.caseStats.maxZ = Math.max(sim.caseStats.maxZ, sim.softPos('bag0')[2]) // after the press
        if (sim.caseStats) sim.caseStats.peakSpeed = Math.max(sim.caseStats.peakSpeed ?? 0, sim.softSpeed('bag0'))
        return [sampleKeys(keys, t), null]
      },
      check(sim) {
        const [x, , z] = sim.softPos('bag0')
        const bag = sim.soft.byName.bag0
        if (!bag.finite()) return 'bag went non-finite'
        if (sim.caseStats.maxZ > BELT_TOP + 0.06) return `bag came up with the hand after the press (centroid z ${sim.caseStats.maxZ.toFixed(3)})`
        if (x - 0.3 < 0.04) return `bag was not pushed along (moved ${(x - 0.3).toFixed(3)} m)`
        if (z > BELT_TOP + 0.04 || sim.touching[0]) return `bag did not stay on the plate (z ${z.toFixed(3)}, touching ${sim.touching[0]})`
        if (sim.softUp('bag0') < TAG_UP) return `bag no longer reads label-up (${sim.softUp('bag0').toFixed(3)})`
        return bagSane(sim, 'bag0')
      },
      describe: sim => `pushed ${(sim.softPos('bag0')[0] - 0.3).toFixed(3)} m, rests at z ${sim.softPos('bag0')[2].toFixed(3)} with ${(sim.soft.byName.bag0.volumeState().volume * 100).toFixed(0)}% of its volume`,
    },
    {
      // the hand reaches over the bag, closes on it, drags it along the plate to the drop point, opens and
      // moves away. A pinch does not lift a flat poly bag with this hand (the edge slips out as the hand
      // rises; the fitted-box colliders that once "held" it were trapping it); lifting takes a scoop from
      // underneath, which the keyframe script cannot do, so this case checks the squeeze, the carry and a
      // clean release
      name: 'left hand closes on a bag, carries it along the plate and lets go clean',
      duration: 15,
      setup(sim) {
        clearPlan(sim)
        sim.teleportSoft('bag0', [0.3, 0.17, BELT_TOP + BAGS.bag0.half[2] + 0.004], [1, 0, 0, 0], true)
        sim.caseStats = { squeezed: false, lowestVolume: 1 }
      },
      hands(sim, t) {
        const r = sim.readyGrip[0]
        const z = BELT_TOP + BAGS.bag0.half[2] + 0.004 + 0.012, fx = 0.3, fy = 0.17 + BAGS.bag0.half[1] - 0.12
        const keys = [
          [0, ...r, 0, 0], [2.5, ...r, 0, 0], [3.5, fx, fy + 0.09, z + 0.12, 0, 0], [4.5, fx, fy + 0.09, z, 0, 0], [5.5, fx, fy + 0.012, z, 0, 0],
          [6.7, fx, fy + 0.012, z, 0, 0], [7.5, fx, fy + 0.012, z, 0, 1], [8.5, fx, fy + 0.012, z + 0.04, 0, 1], [10.5, 0.34, 0.06, z + 0.04, 0, 1],
          [11.5, 0.34, 0.06, z + 0.02, 0, 1], [12.1, 0.34, 0.06, z + 0.02, 0, 0], [13.5, 0.34, 0.06, z + 0.17, 0, 0], [15, 0.36, 0.25, 1.0, 0, 0],
        ]
        if (sim.caseStats && t > 7.5 && t < 11.5) {
          if (sim.touching[0]) sim.caseStats.squeezed = true
          sim.caseStats.lowestVolume = Math.min(sim.caseStats.lowestVolume, sim.soft.byName.bag0.volumeState().volume)
        }
        if (sim.caseStats) sim.caseStats.peakSpeed = Math.max(sim.caseStats.peakSpeed ?? 0, sim.softSpeed('bag0'))
        return [sampleKeys(keys, t), null]
      },
      check(sim) {
        const [, y, z] = sim.softPos('bag0')
        const bag = sim.soft.byName.bag0
        if (!bag.finite()) return 'bag went non-finite'
        if (!sim.caseStats.squeezed) return 'the closed hand never touched the bag'
        if (y > 0.12) return `bag was not carried (y ${y.toFixed(3)})`
        if (z > BELT_TOP + 0.04 || sim.touching[0]) return `bag stayed on the hand (z ${z.toFixed(3)}, touching ${sim.touching[0]})`
        return bagSane(sim, 'bag0')
      },
      describe: sim => `squeezed to ${(sim.caseStats.lowestVolume * 100).toFixed(0)}% of its volume, ends at ${sim.softPos('bag0').map(v => v.toFixed(3))} with ${(sim.soft.byName.bag0.volumeState().volume * 100).toFixed(0)}%`,
    },
    {
      // a bag on a box must press with its own weight only and come along when the box moves: the belt carries
      // the box, the bag rides on it (lagging some centimetres: the reaction is a fraction of the impulse)
      name: 'a bag riding on a belt-carried box',
      duration: 8,
      setup(sim) {
        clearPlan(sim)
        sim.teleportObject(0, [BELT_X, OUTPUT_START_Y - 0.05, BELT_TOP + SIZES[0][2] + 0.002])
        sim.teleportSoft('bag0', [BELT_X, OUTPUT_START_Y - 0.05, BELT_TOP + 2 * SIZES[0][2] + BAGS.bag0.half[2] + 0.012], [1, 0, 0, 0], true)
        sim.caseStats = { peakSpeed: 0, startY: OUTPUT_START_Y - 0.05 }
      },
      hands(sim) {
        if (sim.caseStats) sim.caseStats.peakSpeed = Math.max(sim.caseStats.peakSpeed, sim.softSpeed('bag0'))
        return [[...sim.readyGrip[0], 0, 0], null]
      },
      check(sim) {
        const [bx, by] = sim.objectPos(0), [sx, sy, sz] = sim.softPos('bag0')
        if (sim.caseStats.startY - by < 0.2) return `box was not carried by the belt (moved ${(sim.caseStats.startY - by).toFixed(3)} m)`
        if (Math.hypot(sx - bx, sy - by) > 0.13) return `bag fell off the box (${Math.hypot(sx - bx, sy - by).toFixed(3)} m off)`
        if (sz < BELT_TOP + 2 * SIZES[0][2] + 0.01) return `bag sank into the box (centroid z ${sz.toFixed(3)})`
        const bag = sim.soft.byName.bag0
        if (!bag.finite() || sim.caseStats.peakSpeed > 2) return 'bag was thrown'
        return null
      },
      describe: sim => `box carried ${(sim.caseStats.startY - sim.objectPos(0)[1]).toFixed(3)} m, bag ${Math.hypot(sim.softPos('bag0')[0] - sim.objectPos(0)[0], sim.softPos('bag0')[1] - sim.objectPos(0)[1]).toFixed(3)} m from its centre`,
    },
    {
      // a box pushed into a bag must move it (by at least a centimetre: a heavy, high-friction bag yields little
      // to a box the hand can only push so hard)
      name: 'left hand pushes a box into a bag',
      duration: 10,
      setup(sim) {
        clearPlan(sim)
        sim.teleportSoft('bag0', [0.4, 0.2, BELT_TOP + BAGS.bag0.half[2] + 0.004], [1, 0, 0, 0], true)
        sim.teleportObject(0, [0.24, 0.2, BELT_TOP + SIZES[0][2] + 0.002])
        sim.caseStats = { peakSpeed: 0 }
      },
      hands(sim, t) {
        const r = sim.readyGrip[0], cy = 0.2, mid = BELT_TOP + SIZES[0][2]
        const keys = [[0, ...r, 0, 0], [2.5, ...r, 0, 0], [4, 0.1, cy, mid + 0.1, 0, 0], [5, 0.1, cy, mid - 0.01, 0, 0], [8, 0.25, cy, mid - 0.01, 0, 0], [9, 0.25, cy, mid + 0.12, 0, 0], [10, ...r, 0, 0]]
        if (sim.caseStats) sim.caseStats.peakSpeed = Math.max(sim.caseStats.peakSpeed, sim.softSpeed('bag0'))
        return [sampleKeys(keys, t), null]
      },
      check(sim) {
        const [sx] = sim.softPos('bag0')
        if (sx - 0.4 < 0.01) return `box did not move the bag (bag moved ${(sx - 0.4).toFixed(3)} m)`
        return bagSane(sim, 'bag0')
      },
      describe: sim => `box moved ${(sim.objectPos(0)[0] - 0.24).toFixed(3)} m, bag ${(sim.softPos('bag0')[0] - 0.4).toFixed(3)} m`,
    },
    {
      // the belt carries a bag with a box on it: the box must come along
      name: 'a box riding on a belt-carried bag',
      duration: 8,
      setup(sim) {
        clearPlan(sim)
        sim.teleportSoft('bag0', [BELT_X, OUTPUT_START_Y - 0.05, BELT_TOP + BAGS.bag0.half[2] + 0.004], [1, 0, 0, 0], true)
        sim.teleportObject(0, [BELT_X, OUTPUT_START_Y - 0.05, BELT_TOP + 2 * BAGS.bag0.half[2] + SIZES[0][2] + 0.012])
        sim.caseStats = { peakSpeed: 0, startY: OUTPUT_START_Y - 0.05 }
      },
      hands(sim) {
        if (sim.caseStats) sim.caseStats.peakSpeed = Math.max(sim.caseStats.peakSpeed, sim.softSpeed('bag0'))
        return [[...sim.readyGrip[0], 0, 0], null]
      },
      check(sim) {
        const [, by, bz] = sim.objectPos(0), [, sy] = sim.softPos('bag0')
        if (sim.caseStats.startY - sy < 0.2) return `bag was not carried by the belt (moved ${(sim.caseStats.startY - sy).toFixed(3)} m)`
        if (Math.abs(by - sy) > 0.05) return `box did not come along on the bag (${Math.abs(by - sy).toFixed(3)} m behind)`
        if (bz < BELT_TOP + 2 * BAGS.bag0.half[2]) return `box fell off the bag (z ${bz.toFixed(3)})`
        const bag = sim.soft.byName.bag0
        if (!bag.finite() || sim.caseStats.peakSpeed > 2) return 'bag was thrown'
        return null
      },
      describe: sim => `bag carried ${(sim.caseStats.startY - sim.softPos('bag0')[1]).toFixed(3)} m, box ${Math.abs(sim.objectPos(0)[1] - sim.softPos('bag0')[1]).toFixed(3)} m from it`,
    },
    {
      // the box must rest on the bag: not sink, not slide off, not bounce, and over the last second move no
      // more than the explicit coupling leaves it (it keeps millimetre jitter and may creep slowly)
      name: 'a box dropped on a bag rests on it',
      duration: 6,
      setup(sim) {
        clearPlan(sim)
        sim.teleportSoft('bag0', [0.35, 0.15, BELT_TOP + BAGS.bag0.half[2] + 0.004], [1, 0, 0, 0], true)
        sim.teleportObject(0, [0.35, 0.15, BELT_TOP + 2 * BAGS.bag0.half[2] + SIZES[0][2] + 0.01])
        sim.caseStats = { speedSum: 0, speedN: 0, zMin: Infinity, zMax: -Infinity }
      },
      hands(sim, t) {
        if (sim.caseStats && t > 7) { // the last second
          const st = sim.caseStats, z = sim.objectPos(0)[2]
          st.speedSum += sim.objectSpeed(0); st.speedN++; st.zMin = Math.min(st.zMin, z); st.zMax = Math.max(st.zMax, z)
        }
        return [[...sim.readyGrip[0], 0, 0], null]
      },
      check(sim) {
        const z = sim.objectPos(0)[2] - SIZES[0][2]
        const st = sim.caseStats, meanSpeed = st.speedSum / Math.max(1, st.speedN)
        if (z < BELT_TOP + 0.015) return `box sank through the bag (bottom ${z.toFixed(3)})`
        if (Math.hypot(sim.objectPos(0)[0] - 0.35, sim.objectPos(0)[1] - 0.15) > 0.04) return `box slid off the bag`
        if (meanSpeed > 0.1) return `box never came to rest (mean ${meanSpeed.toFixed(3)} m/s over the last second)`
        if (st.zMax - st.zMin > 0.005) return `box bounces on the bag (${((st.zMax - st.zMin) * 1000).toFixed(1)} mm over the last second)`
        return bagSane(sim, 'bag0')
      },
      describe: sim => `box bottom ${(sim.objectPos(0)[2] - SIZES[0][2] - BELT_TOP).toFixed(3)} above the plate, ${(sim.caseStats.speedSum / Math.max(1, sim.caseStats.speedN)).toFixed(3)} m/s and ${((sim.caseStats.zMax - sim.caseStats.zMin) * 1000).toFixed(1)} mm of height over the last second`,
    },
    {
      name: 'a bag dropped on a bag stacks or slides off, never merges',
      duration: 10,
      setup(sim) {
        clearPlan(sim)
        sim.teleportSoft('bag0', [0.35, 0.15, BELT_TOP + BAGS.bag0.half[2] + 0.004], [1, 0, 0, 0], true)
        sim.teleportSoft('bag1', [0.37, 0.17, BELT_TOP + 2 * BAGS.bag0.half[2] + BAGS.bag1.half[2] + 0.02], [Math.cos(0.15), 0, 0, Math.sin(0.15)], true)
        sim.caseStats = { peakSpeed: 0 }
      },
      hands(sim, t) {
        if (sim.caseStats) {
          const st = sim.caseStats
          st.peakSpeed = Math.max(st.peakSpeed ?? 0, sim.softSpeed('bag0'), sim.softSpeed('bag1'))
          if (t > 11) { st.speedSum = (st.speedSum ?? 0) + Math.max(sim.softSpeed('bag0'), sim.softSpeed('bag1')); st.speedN = (st.speedN ?? 0) + 1 } // the last second
        }
        return [[...sim.readyGrip[0], 0, 0], null]
      },
      check(sim) {
        const a = sim.soft.byName.bag0, b = sim.soft.byName.bag1
        a.updateDop(); b.updateDop()
        const overlap = Math.max(sim.soft.overlap(a, b), sim.soft.overlap(b, a))
        if (overlap > 0.05) return `bags merged (${(overlap * 100).toFixed(0)}% of one bag's particles inside the other)`
        const meanSpeed = (sim.caseStats.speedSum ?? 0) / Math.max(1, sim.caseStats.speedN ?? 0)
        if (meanSpeed > 0.05) return `bags never settled (${meanSpeed.toFixed(3)} m/s mean over the last second)`
        return bagSane(sim, 'bag0') ?? bagSane(sim, 'bag1')
      },
      describe: sim => `upper bag centroid z ${sim.softPos('bag1')[2].toFixed(3)} (${sim.softPos('bag1')[2] > BELT_TOP + 0.055 ? 'stacked' : 'slid off onto the plate'})`,
    },
  ],

  // Headless check: packages already turned label-up on the output belt; the belt must carry them off and score them
  solved(sim) {
    const st = sim.taskState
    st.active = []
    st.order.forEach((plan, n) => {
      const it = ITEMS[plan.item]
      itemTeleport(sim, it, onBelt(it, n), [1, 0, 0, 0])
      st.active.push({ item: plan.item, index: n })
    })
    st.spawned = st.order.length
  },
}
