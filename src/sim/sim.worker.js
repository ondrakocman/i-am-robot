// Physics worker: runs MuJoCo (WASM) in real time off the render thread.
//   main -> worker: init, input (operator hands, every XR frame), abort (XR session ended / recentered)
//   worker -> main: ready (scene description for rendering), state (body poses ~120 Hz), episode, error

import loadMujoco from '@mujoco/mujoco'
import { loadScene } from './loadScene.js'
import { TaskSim, COMMON } from './TaskSim.js'
import { getTask } from './tasks/index.js'
import { encodeEpisode } from './episode.js'

const TICK_MS = 4
const MAX_CATCHUP_MS = 20     // per tick; beyond this the sim runs slower than real time instead of freezing
const POST_INTERVAL_MS = 8    // body poses while running
const IDLE_POST_INTERVAL_MS = 50
const INPUT_TIMEOUT_MS = 150  // no input message for this long (headset off, tab hidden) = hands untracked
const GEOM_TYPES = ['plane', 'hfield', 'sphere', 'capsule', 'ellipsoid', 'cylinder', 'box', 'mesh'] // mjtGeom order

let mj = null
let sim = null
let owed = 0
let lastTick = 0
let lastPost = 0
let lastInput = -Infinity
let bodies = null             // reused between posts when the previous buffer has been returned
const perf = { windowStart: 0, simTime: 0, stepMs: 0, steps: 0, rtf: 1, msPerStep: 0 }

self.onmessage = ({ data: msg }) => {
  switch (msg.type) {
    case 'init':
      init(msg).catch(err => self.postMessage({ type: 'error', message: String(err?.stack ?? err) }))
      break
    case 'input':
      sim?.setInput(msg.input, msg.raw)
      lastInput = performance.now()
      break
    case 'abort':
      sim?.abort(msg.outcome)
      sim?.clearInput()
      break
    case 'state-buffer':
      bodies = msg.bodies
      break
    case 'meta': // extra header fields known only to the main thread (XR frame rate once a session exists)
      if (sim) sim.meta = { ...sim.meta, ...msg.meta }
      break
  }
}

async function init({ baseUrl, timestep, autopilot, session, task: taskName, appVersion, userAgent }) {
  const task = getTask(taskName)
  mj = await loadMujoco()
  const readFile = async path => {
    const res = await fetch(baseUrl + path)
    if (!res.ok) throw new Error(`fetch ${path}: ${res.status}`)
    return path.endsWith('.xml') ? res.text() : new Uint8Array(await res.arrayBuffer())
  }
  const { model: m, assets } = await loadScene(mj, readFile, { scene: task.scene, timestep })
  sim = new TaskSim(mj, m, task, {
    autopilot,
    meta: { session, app_version: appVersion, mujoco: __MUJOCO_VERSION__, assets, user_agent: userAgent },
    onEpisode: ({ header, frames }) => {
      // encode copies the frames once; the Blob crosses to the main thread without another copy
      self.postMessage({ type: 'episode', header, data: new Blob([encodeEpisode(header, frames)]) })
    },
  })

  const { scene, transfer } = describeScene(m)
  self.postMessage({
    type: 'ready',
    scene,
    eye: sim.eyePosition(),
    // palm site relative to the wrist body, per hand: where the renderer measures the ghost-hand gap
    palmOffset: sim.arms.map(a => [0, 1, 2].map(k => m.site_pos[3 * a.palmSite + k])),
    task: {
      name: task.name, instruction: task.instruction, title: task.title ?? task.instruction, objects: task.objects,
      resetButton: COMMON.resetButton, materials: task.materials ?? {}, geometry: task.geometry ?? {},
    },
    timestep: sim.dt,
  }, transfer)
  lastTick = perf.windowStart = performance.now()
  const interval = setInterval(() => {
    try {
      tick()
    } catch (err) {
      // a task bug or embind abort: keep what was recorded, report once, and stop instead of throwing every 4 ms
      clearInterval(interval)
      try { if (sim.status === 'running') sim.endEpisode('error') } catch { /* the data itself may be gone */ }
      self.postMessage({ type: 'error', message: String(err?.stack ?? err) })
    }
  }, TICK_MS)
}

function tick() {
  const now = performance.now()
  owed = Math.min(owed + (now - lastTick) / 1000, 0.1)
  lastTick = now
  if (now - lastInput > INPUT_TIMEOUT_MS) sim.clearInput()

  if (sim.status === 'waiting') {
    sim.step() // physics frozen; the controller only watches for the operator's hands
    owed = 0
    // the real-time window measures stepping only: restart it so idle time doesn't count as slow physics
    perf.windowStart = now
    perf.simTime = perf.stepMs = perf.steps = 0
  } else {
    const dt = sim.dt
    while (owed >= dt) {
      const t0 = performance.now()
      sim.step()
      perf.stepMs += performance.now() - t0
      perf.steps++
      perf.simTime += dt
      owed -= dt
      if (performance.now() - now > MAX_CATCHUP_MS) { owed = 0; break }
    }
  }

  if (now - perf.windowStart >= 1000) {
    const wall = (now - perf.windowStart) / 1000
    perf.rtf = perf.steps ? perf.simTime / wall : 1
    perf.msPerStep = perf.steps ? perf.stepMs / perf.steps : 0
    perf.windowStart = now
    perf.simTime = perf.stepMs = perf.steps = 0
    sim.reportRealtime(perf.rtf)
  }

  if (now - lastPost >= (sim.status === 'waiting' ? IDLE_POST_INTERVAL_MS : POST_INTERVAL_MS)) {
    lastPost = now
    const out = sim.writeBodies(bodies ?? new Float32Array(7 * sim.m.nbody))
    bodies = null
    self.postMessage({
      type: 'state',
      bodies: out,
      info: {
        status: sim.status,
        episode: sim.episode,
        elapsed: sim.status === 'waiting' ? 0 : (sim.status === 'running' ? sim.d.time : sim.endTime) - sim.startTime,
        touching: [sim.touching[0], sim.touching[1]],
        resetProgress: Math.min(1, sim.resetTimer / COMMON.resetHold),
        taskLine: sim.task.hud ? sim.task.hud(sim) : '',
        rtf: perf.rtf,
        msPerStep: perf.msPerStep,
      },
    }, [out.buffer])
  }
}

// Everything the main thread needs to draw the model: bodies, visible geoms (robot collision meshes in
// group 3 and sites are skipped) with their MuJoCo material names, and the mesh data they reference.
function describeScene(m) {
  const name = (type, i) => mj.mj_id2name(m, mj.mjtObj[type].value, i) ?? ''
  const MESH = mj.mjtGeom.mjGEOM_MESH.value
  const bodies = Array.from({ length: m.nbody }, (_, b) => ({ name: name('mjOBJ_BODY', b), robot: sim.isRobotBody(b) }))
  const geoms = []
  const meshes = {}
  const transfer = []
  for (let g = 0; g < m.ngeom; g++) {
    if (m.geom_group[g] > 2) continue
    const type = m.geom_type[g]
    const mat = m.geom_matid[g]
    const rgba = Array.from(mat >= 0 ? m.mat_rgba.slice(4 * mat, 4 * mat + 4) : m.geom_rgba.slice(4 * g, 4 * g + 4))
    if (rgba[3] === 0) continue
    let mesh = -1
    if (type === MESH) {
      mesh = m.geom_dataid[g]
      if (!meshes[mesh]) {
        const va = m.mesh_vertadr[mesh], vn = m.mesh_vertnum[mesh]
        const fa = m.mesh_faceadr[mesh], fn = m.mesh_facenum[mesh]
        const vert = m.mesh_vert.slice(3 * va, 3 * (va + vn))
        const face = m.mesh_face.slice(3 * fa, 3 * (fa + fn))
        meshes[mesh] = { name: name('mjOBJ_MESH', mesh), vert, face }
        transfer.push(vert.buffer, face.buffer)
      }
    }
    geoms.push({
      name: name('mjOBJ_GEOM', g),
      body: m.geom_bodyid[g],
      type: GEOM_TYPES[type] ?? 'other',
      size: Array.from(m.geom_size.slice(3 * g, 3 * g + 3)),
      pos: Array.from(m.geom_pos.slice(3 * g, 3 * g + 3)),
      quat: Array.from(m.geom_quat.slice(4 * g, 4 * g + 4)),
      rgba,
      material: mat >= 0 ? name('mjOBJ_MATERIAL', mat) : '',
      mesh,
    })
  }
  return { scene: { bodies, geoms, meshes }, transfer }
}
