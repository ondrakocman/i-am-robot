import { useEffect, useRef, useState } from 'react'
import { useFrame, useThree } from '@react-three/fiber'
import * as THREE from 'three'
import { RoomEnvironment } from 'three/examples/jsm/environments/RoomEnvironment.js'
import { retargetHand, RetargetingFilter } from '../systems/HandRetargeting.js'
import { QuaternionSmoother } from '../systems/Smoothing.js'
import { OneEuroVector3 } from '../systems/OneEuroFilter.js'
import { XR_JOINT_NAMES, ROBOT_BASE_QUAT, XR_TO_URDF_L, XR_TO_URDF_R } from '../constants/kinematics.js'
import { HAND_INPUT, INPUT_SIZE, RAW_SIZE } from '../sim/TaskSim.js'
import { saveEpisode, onEpisodesChanged, requestPersistence } from '../sim/episodeStore.js'

const params = new URLSearchParams(location.search)
const SESSION_ID = crypto.randomUUID?.() ?? String(Date.now())

// G1 colors: brushed-aluminium shell on the body and arms (Unitree's URDF "white" material), dark pelvis,
// hip-pitch housings, feet, head and logo (URDF "dark"), and black Dex3 hands with rubber fingertip pads.
const MAT_BODY = new THREE.MeshStandardMaterial({ color: 0x9c9fa3, roughness: 0.42, metalness: 0.7 })
const MAT_ACCENT = new THREE.MeshStandardMaterial({ color: 0x2a2b2e, roughness: 0.6, metalness: 0.3 })
const MAT_PAD = new THREE.MeshStandardMaterial({ color: 0x15161a, roughness: 0.95, metalness: 0 })
const DARK_BODY = /^pelvis$|_hip_pitch_link$|_ankle_roll_link$|_hand_/
const DARK_MESH = /^(head_link|logo_link)$|_hand_palm_link$/   // palm mesh hangs off the (silver) wrist body
const ROBOT_BODY = /_link$|^pelvis$/
const PAD_BODY = /_hand_(thumb_2|index_1|middle_1)_link$/
const SHADOW_CASTER_BODY = /elbow|wrist|hand/
// Surface look per MuJoCo material name (colors come from the MJCF); anything else is a matte default
const MATERIAL_LOOK = {
  floor: { roughness: 0.95 },
  table: { roughness: 0.75 },
  table_leg: { roughness: 0.8 },
  box: { roughness: 0.85 },                                 // matte plastic
  tube: { color: 0xb4b8bd, roughness: 0.32, metalness: 1 },  // brushed steel
  cardboard: { roughness: 0.95 },
  tag: { roughness: 0.6 },
  tagbar: { roughness: 0.6 },
  roller: { roughness: 0.4, metalness: 0.6 },
  rail: { roughness: 0.45, metalness: 0.7 },
  leg: { roughness: 0.6, metalness: 0.5 },
  stop: { roughness: 0.6 },
  red: { roughness: 0.55 }, green: { roughness: 0.55 }, blue: { roughness: 0.55 },        // printed PLA
  red_bin: { roughness: 0.7 }, green_bin: { roughness: 0.7 }, blue_bin: { roughness: 0.7 },
}
// Visible room around the robot (robot frame: x forward, z up); visual only, nothing collides with it.
// The box bottom sits 2 cm under the MuJoCo floor plane so the two don't z-fight.
const ROOM = { size: [7, 7, 2.9], center: [0.8, 0, 1.43], wall: 0xcfd3d6, lightPanel: [0.45, 0, 2.87] }
const GHOST_SHOW_AT = 0.02   // m between the operator's wrist and the robot palm before the ghost appears
const GHOST_FULL_AT = 0.06
const GHOST_CHAINS = [
  ['wrist', 'thumb-metacarpal', 'thumb-phalanx-proximal', 'thumb-phalanx-distal', 'thumb-tip'],
  ['wrist', 'index-finger-metacarpal', 'index-finger-phalanx-proximal', 'index-finger-phalanx-intermediate', 'index-finger-phalanx-distal', 'index-finger-tip'],
  ['wrist', 'middle-finger-metacarpal', 'middle-finger-phalanx-proximal', 'middle-finger-phalanx-intermediate', 'middle-finger-phalanx-distal', 'middle-finger-tip'],
]
const PALM_OFFSET = [new THREE.Vector3(0.0415, 0.003, 0), new THREE.Vector3(0.0415, -0.003, 0)]
const TOUCH_EMISSIVE = new THREE.Color(0x0e4a26)
const NO_EMISSIVE = new THREE.Color(0x000000)
const CORRECTION = [XR_TO_URDF_L, XR_TO_URDF_R]
const RAW_HAND = 25 * 7
const HAND_DROPOUT_S = 0.2   // tracking gap after which the filters restart from the new pose
const HUD_HZ = 4

// three.js plane facing the robot (normal -x in the MuJoCo frame), text running toward the robot's right
const FACING_ROBOT = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(
  new THREE.Vector3(0, -1, 0), new THREE.Vector3(0, 0, 1), new THREE.Vector3(-1, 0, 0),
))

const _v = new THREE.Vector3()
const _q = new THREE.Quaternion()
const _inv = new THREE.Matrix4()
const _rootQinv = new THREE.Quaternion()
const _eye = new THREE.Vector3()
const _palm = new THREE.Vector3()
const _headPos = new THREE.Vector3()
const _headQuat = new THREE.Quaternion()
const _dummy = new THREE.Object3D()

export function MujocoScene() {
  const { gl, camera, scene } = useThree()
  const worldRef = useRef()
  const workerRef = useRef(null)
  const [world, setWorld] = useState(null)
  const latest = useRef(null)
  const applied = useRef(null)
  // What the HUD knows about storage: committed counts plus in-flight / failed saves
  const saves = useRef({ total: 0, success: 0, pending: 0, failed: 0, persistent: null, error: null })
  const xr = useRef({ session: null, refSpace: null, calibrated: false, hands: [newHandState(), newHandState()] })
  const input = useRef({ input: new Float32Array(INPUT_SIZE), raw: new Float32Array(RAW_SIZE) })

  useEffect(() => {
    const worker = new Worker(new URL('../sim/sim.worker.js', import.meta.url), { type: 'module' })
    workerRef.current = worker
    setStatusText('Loading physics…')
    const fail = message => {
      console.error('[sim]', message)
      saves.current.error = 'PHYSICS ERROR — reload the page'
      setStatusText('Physics failed: ' + message.split('\n')[0])
    }
    worker.onmessage = ({ data }) => {
      if (data.type === 'state') {
        latest.current = data
      } else if (data.type === 'ready') {
        setWorld(buildWorld(data))
        setStatusText(`Physics ready · MuJoCo ${data.timestep * 1000} ms step`)
      } else if (data.type === 'episode') {
        saves.current.pending++
        saveEpisode(data.header, data.buffer)
          .catch(err => { saves.current.failed++; console.error('[episodes] save failed', err) })
          .finally(() => { saves.current.pending-- })
      } else if (data.type === 'error') {
        fail(data.message)
      }
    }
    worker.onerror = e => fail(e.message ?? String(e))
    worker.postMessage({
      type: 'init',
      baseUrl: new URL(import.meta.env.BASE_URL, location.href).href,
      timestep: Number(params.get('dt')) || undefined,
      autopilot: params.has('autopilot'),
      session: SESSION_ID,
      task: params.get('task') || undefined,
      appVersion: __GIT_SHA__,
    })
    const unsubscribe = onEpisodesChanged(s => Object.assign(saves.current, s))
    requestPersistence().then(ok => { saves.current.persistent = ok }).catch(() => { saves.current.persistent = false })
    return () => { worker.terminate(); unsubscribe() }
  }, [])

  // Image-based lighting from a procedural room: no texture download, generated once at startup
  useEffect(() => {
    const pmrem = new THREE.PMREMGenerator(gl)
    const env = pmrem.fromScene(new RoomEnvironment(), 0.04).texture
    pmrem.dispose()
    // The scene object is three.js state, not React state; assigning its environment is the supported way
    /* eslint-disable react-hooks/immutability */
    scene.environment = env
    scene.environmentIntensity = 0.55
    return () => { scene.environment = null; env.dispose() }
    /* eslint-enable react-hooks/immutability */
  }, [gl, scene])

  // Desktop preview camera: front-left of the robot, or the operator's view with ?view=eye.
  // WebXR overrides it with the headset pose.
  useEffect(() => {
    if (gl.xr.isPresenting || !world) return
    if (params.get('view') === 'eye') {
      world.root.updateWorldMatrix(true, false)
      camera.position.copy(world.root.localToWorld(new THREE.Vector3().fromArray(world.eye)))
      camera.lookAt(world.root.localToWorld(new THREE.Vector3(0.45, 0, 0.75)))
    } else {
      camera.position.set(-0.85, 1.5, -1.35)
      camera.lookAt(0, 0.9, -0.3)
    }
  }, [camera, gl, world])

  useEffect(() => {
    const hideHead = params.get('view') === 'eye'
    if (world) world.headMeshes.forEach(m => { m.visible = !hideHead })
  }, [world])

  useFrame((_state, delta, xrFrame) => {
    if (!world) return
    const s = latest.current
    if (s && s !== applied.current) {
      applied.current = s
      applyBodies(world, s.bodies)
      workerRef.current?.postMessage({ type: 'state-buffer', bodies: s.bodies }, [s.bodies.buffer])
      applyInfo(world, s.info, saves.current)
    }
    if (!xrFrame) return

    const session = gl.xr.getSession()
    const refSpace = gl.xr.getReferenceSpace()
    if (!session || !refSpace) return
    const st = xr.current
    if (st.session !== session) {
      st.session = session
      st.calibrated = false
      st.hands.forEach(h => { h.lastSeen = -Infinity })
      // Leaving VR ends the running episode; the worker's input watchdog also stops the hands
      session.addEventListener('end', () => { workerRef.current?.postMessage({ type: 'abort' }) }, { once: true })
    }
    if (st.refSpace !== refSpace) {
      st.refSpace = refSpace
      st.calibrated = false
      // Quest "recenter": the reference space moves, so the world must be placed again
      refSpace.addEventListener('reset', () => { st.calibrated = false })
    }

    const viewer = xrFrame.getViewerPose(refSpace)
    if (!viewer) return // no tracking this frame; keep the last input until the worker's watchdog clears it
    const { position: hp, orientation: hq } = viewer.transform
    _headPos.set(hp.x, hp.y, hp.z)
    _headQuat.set(hq.x, hq.y, hq.z, hq.w)
    if (!st.calibrated) {
      _eye.fromArray(world.eye)
      calibrate(_headPos, _headQuat, worldRef.current, world.root, _eye)
      st.calibrated = true
    }

    const { input: inp, raw } = input.current
    readOperator(xrFrame, session, refSpace, world, st.hands, delta, inp, raw)
    workerRef.current?.postMessage({ type: 'input', input: inp, raw })
  })

  return (
    <>
      <hemisphereLight args={['#c8d8e8', '#4a4540', 0.9]} />
      <group ref={worldRef}>
        {world && <primitive object={world.root} />}
      </group>
      {world && <primitive object={world.ghostGroup} />}
    </>
  )
}

// ── Scene construction ──────────────────────────────────────────────────────

function buildWorld({ scene, eye, task }) {
  const root = new THREE.Group()
  root.quaternion.copy(ROBOT_BASE_QUAT)

  const bodies = scene.bodies.map(name => {
    const g = new THREE.Group()
    g.name = name
    root.add(g)
    return g
  })
  // Per hand: [shell, accent, pad] clones so the touch glow can tint one hand at a time
  const handMaterials = [0, 1].map(() => [MAT_BODY.clone(), MAT_ACCENT.clone(), MAT_PAD.clone()])
  const headMeshes = []
  const meshCache = new Map()
  const materialCache = new Map()

  const objects = new Set(task.objects)
  for (const geom of scene.geoms) {
    const bodyName = scene.bodies[geom.body]
    const meshName = geom.mesh >= 0 ? scene.meshes[geom.mesh].name : ''
    const isRobot = ROBOT_BODY.test(bodyName)
    let material
    if (!isRobot) {
      const key = geom.material + '|' + geom.rgba.join(',') + (geom.mesh >= 0 ? '|flat' : '')
      if (!materialCache.has(key)) {
        const [r, g, b, a] = geom.rgba
        materialCache.set(key, new THREE.MeshStandardMaterial({
          color: new THREE.Color().setRGB(r, g, b, THREE.SRGBColorSpace),
          roughness: 0.6, transparent: a < 1, opacity: a, flatShading: geom.mesh >= 0,
          ...(MATERIAL_LOOK[geom.material] ?? {}),
        }))
      }
      material = materialCache.get(key)
    } else {
      const hand = bodyName.startsWith('left_hand_') || bodyName === 'left_wrist_yaw_link' ? 0
        : bodyName.startsWith('right_hand_') || bodyName === 'right_wrist_yaw_link' ? 1 : -1
      const kind = PAD_BODY.test(bodyName) ? 2 : DARK_BODY.test(bodyName) || DARK_MESH.test(meshName) ? 1 : 0
      material = hand >= 0 ? handMaterials[hand][kind] : [MAT_BODY, MAT_ACCENT, MAT_PAD][kind]
    }
    const mesh = new THREE.Mesh(geomGeometry(geom, scene.meshes, meshCache), material)
    mesh.position.fromArray(geom.pos)
    mesh.quaternion.set(geom.quat[1], geom.quat[2], geom.quat[3], geom.quat[0])
    // One shadow pass: only the forearms, hands and task objects cast; the scene receives
    mesh.castShadow = objects.has(bodyName) || SHADOW_CASTER_BODY.test(bodyName)
    mesh.receiveShadow = !isRobot
    if (meshName === 'head_link') headMeshes.push(mesh)
    bodies[geom.body].add(mesh)
  }

  // Key light inside the robot frame (z up) so its shadow camera follows the scene after VR calibration
  const sun = new THREE.DirectionalLight(0xfff4e6, 2.6)
  sun.position.set(-0.8, 0.9, 2.6)
  sun.target.position.set(0.45, 0, 0.8)
  sun.castShadow = true
  sun.shadow.mapSize.set(1024, 1024)
  sun.shadow.camera.left = sun.shadow.camera.bottom = -1.1
  sun.shadow.camera.right = sun.shadow.camera.top = 1.1
  sun.shadow.camera.near = 0.5
  sun.shadow.camera.far = 5
  sun.shadow.bias = -0.0004
  sun.shadow.normalBias = 0.02
  root.add(sun, sun.target)
  const fill = new THREE.DirectionalLight(0xdde8ff, 0.7)
  fill.position.set(1.5, -1.5, 1.8)
  fill.target.position.set(0.45, 0, 0.8)
  root.add(fill, fill.target)

  const room = new THREE.Mesh(
    new THREE.BoxGeometry(...ROOM.size),
    new THREE.MeshStandardMaterial({ color: ROOM.wall, roughness: 0.9, metalness: 0, side: THREE.BackSide }),
  )
  room.position.fromArray(ROOM.center)
  room.receiveShadow = true
  root.add(room)
  const panel = new THREE.Mesh(
    new THREE.PlaneGeometry(1.2, 0.6),
    new THREE.MeshBasicMaterial({ color: 0xfff8ee, toneMapped: false }),
  )
  panel.position.fromArray(ROOM.lightPanel)
  panel.rotation.x = Math.PI // face down
  root.add(panel)

  const ghostGroup = new THREE.Group()
  const ghosts = [makeGhostHand(), makeGhostHand()]
  ghosts.forEach(g => ghostGroup.add(g.group))
  const wristBodies = ['left', 'right'].map(side => bodies[scene.bodies.indexOf(`${side}_wrist_yaw_link`)])

  const hud = makeHud(task.title)
  hud.mesh.position.set(0.8, 0, 1.04)
  hud.mesh.quaternion.copy(FACING_ROBOT)
  root.add(hud.mesh)

  const reset = makeResetButton()
  reset.group.position.fromArray(task.resetButton)
  root.add(reset.group)

  return { root, bodies, eye, handMaterials, headMeshes, hud, reset, ghostGroup, ghosts, wristBodies }
}

const MJ_GEOM = { PLANE: 0, SPHERE: 2, CAPSULE: 3, ELLIPSOID: 4, CYLINDER: 5, BOX: 6, MESH: 7 }

function geomGeometry(g, meshes, cache) {
  const [s0, s1, s2] = g.size
  switch (g.type) {
    case MJ_GEOM.PLANE: return new THREE.PlaneGeometry(s0 > 0 ? 2 * s0 : 30, s1 > 0 ? 2 * s1 : 30)
    case MJ_GEOM.SPHERE: return new THREE.SphereGeometry(s0, 24, 16)
    case MJ_GEOM.CAPSULE: return new THREE.CapsuleGeometry(s0, 2 * s1, 8, 16).rotateX(Math.PI / 2)
    case MJ_GEOM.ELLIPSOID: return new THREE.SphereGeometry(1, 24, 16).scale(s0, s1, s2)
    case MJ_GEOM.CYLINDER: {
      if (g.material !== 'tube') return new THREE.CylinderGeometry(s0, s0, 2 * s1, 32).rotateX(Math.PI / 2)
      // Hollow tube with a 2 mm wall; the physics has a matching ring of thin boxes (see tube_box.xml)
      const ri = s0 - 0.002
      const profile = [[ri, -s1], [s0, -s1], [s0, s1], [ri, s1], [ri, -s1]].map(([x, y]) => new THREE.Vector2(x, y))
      return new THREE.LatheGeometry(profile, 48).rotateX(Math.PI / 2)
    }
    case MJ_GEOM.BOX: return new THREE.BoxGeometry(2 * s0, 2 * s1, 2 * s2)
    case MJ_GEOM.MESH: {
      if (!cache.has(g.mesh)) {
        const { vert, face } = meshes[g.mesh]
        const geo = new THREE.BufferGeometry()
        geo.setAttribute('position', new THREE.BufferAttribute(vert, 3))
        // MuJoCo faces are Int32; WebGL index buffers must be unsigned
        geo.setIndex(new THREE.BufferAttribute(new Uint32Array(face.buffer, face.byteOffset, face.length), 1))
        geo.computeVertexNormals() // smooth normals suit the decimated robot; scene meshes (bins) use flatShading
        cache.set(g.mesh, geo)
      }
      return cache.get(g.mesh)
    }
    default: return new THREE.BufferGeometry()
  }
}

function applyBodies(world, b) {
  for (let i = 0; i < world.bodies.length; i++) {
    const o = 7 * i
    const g = world.bodies[i]
    g.position.set(b[o], b[o + 1], b[o + 2])
    g.quaternion.set(b[o + 4], b[o + 5], b[o + 6], b[o + 3])
  }
}

function applyInfo(world, info, saves) {
  for (let s = 0; s < 2; s++) {
    for (const mat of world.handMaterials[s]) mat.emissive.copy(info.touching[s] ? TOUCH_EMISSIVE : NO_EMISSIVE)
  }
  world.reset.setProgress(info.resetProgress)
  world.hud.draw(info, saves)
}

// ── Ghost hand: the operator's real hand, shown only when the robot hand can't follow it ────────────

function makeGhostHand() {
  const group = new THREE.Group()
  const segments = GHOST_CHAINS.reduce((n, c) => n + c.length - 1, 0)
  const lineGeo = new THREE.BufferGeometry()
  lineGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(segments * 6), 3))
  const lineMat = new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0, depthTest: false })
  const lines = new THREE.LineSegments(lineGeo, lineMat)
  lines.frustumCulled = false
  const jointNames = [...new Set(GHOST_CHAINS.flat())]
  const dotMat = new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0, depthTest: false })
  const dots = new THREE.InstancedMesh(new THREE.SphereGeometry(0.004, 6, 5), dotMat, jointNames.length)
  dots.frustumCulled = false
  group.add(lines, dots)
  group.visible = false
  group.renderOrder = 10
  return {
    group,
    update(joints, strength) {
      group.visible = strength > 0
      if (!group.visible) return
      lineMat.opacity = 0.75 * strength
      dotMat.opacity = 0.9 * strength
      const pos = lineGeo.attributes.position
      let i = 0
      for (const chain of GHOST_CHAINS) {
        for (let k = 0; k + 1 < chain.length; k++) {
          const a = joints[chain[k]]?.position, b = joints[chain[k + 1]]?.position
          if (!a || !b) continue
          pos.setXYZ(i++, a.x, a.y, a.z)
          pos.setXYZ(i++, b.x, b.y, b.z)
        }
      }
      pos.needsUpdate = true
      jointNames.forEach((n, k) => {
        const p = joints[n]?.position
        _dummy.position.copy(p ?? lines.position)
        _dummy.scale.setScalar(p ? (n === 'wrist' ? 2 : 1) : 0)
        _dummy.updateMatrix()
        dots.setMatrixAt(k, _dummy.matrix)
      })
      dots.instanceMatrix.needsUpdate = true
    },
  }
}

// ── In-scene UI ─────────────────────────────────────────────────────────────

const STATUS_TEXT = {
  waiting: ['RAISE YOUR HANDS TO START', '#9fb4c8'],
  running: ['RECORDING', '#ff5d5d'],
  success: ['SUCCESS', '#5dff8f'],
  partial: ['DONE — NOT ALL CORRECT', '#ffb35d'],
  dropped: ['DROPPED — RESETTING', '#ffb35d'],
  timeout: ['TIMEOUT — RESETTING', '#ffb35d'],
  aborted: ['RESET', '#9fb4c8'],
}

function makeHud(instruction) {
  const canvas = document.createElement('canvas')
  canvas.width = 1024
  canvas.height = 512
  const ctx = canvas.getContext('2d')
  const texture = new THREE.CanvasTexture(canvas)
  texture.colorSpace = THREE.SRGBColorSpace
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(0.44, 0.22),
    new THREE.MeshBasicMaterial({ map: texture, transparent: true, toneMapped: false }),
  )
  let last = ''
  let lastDraw = 0
  const draw = (info, saves) => {
    const now = performance.now()
    if (now - lastDraw < 1000 / HUD_HZ) return
    const [label, color] = saves.error ? [saves.error, '#ff5d5d'] : STATUS_TEXT[info.status] ?? [info.status.toUpperCase(), '#ffffff']
    const storage = saves.failed ? `SAVE FAILED ×${saves.failed}` : saves.pending ? 'saving…' : `saved ${saves.total} (${saves.success} ok)`
    const lines = [
      label + (info.status === 'running' ? `  ${Math.floor(info.elapsed)} s` : ''),
      (info.taskLine ? `${info.taskLine}   ·   ` : '') + `episode ${info.episode}   ${storage}`,
      `physics ${info.rtf.toFixed(2)}× real time · ${info.msPerStep.toFixed(2)} ms/step` + (saves.persistent === false ? ' · storage not persistent' : ''),
    ]
    const key = lines.join('|') + color
    if (key === last) return
    last = key
    lastDraw = now
    ctx.clearRect(0, 0, 1024, 512)
    ctx.fillStyle = 'rgba(10, 16, 24, 0.78)'
    ctx.beginPath()
    ctx.roundRect(8, 8, 1008, 496, 36)
    ctx.fill()
    ctx.fillStyle = '#ffffff'
    ctx.font = '600 54px system-ui, sans-serif'
    ctx.fillText(instruction, 56, 112)
    ctx.fillStyle = color
    ctx.font = '700 64px system-ui, sans-serif'
    ctx.fillText(lines[0], 56, 228)
    ctx.fillStyle = saves.failed ? '#ff5d5d' : '#c8d4e0'
    ctx.font = '400 40px system-ui, sans-serif'
    ctx.fillText(lines[1], 56, 338)
    ctx.fillStyle = info.rtf < 0.95 || saves.persistent === false ? '#ffb35d' : '#7f93a6'
    ctx.font = '400 34px system-ui, sans-serif'
    ctx.fillText(lines[2], 56, 432)
    texture.needsUpdate = true
  }
  return { mesh, draw }
}

function makeResetButton() {
  const group = new THREE.Group()
  const material = new THREE.MeshStandardMaterial({ color: 0xc0392b, roughness: 0.4, emissive: 0x220000 })
  group.add(new THREE.Mesh(new THREE.SphereGeometry(0.03, 24, 16), material))

  const canvas = document.createElement('canvas')
  canvas.width = 256
  canvas.height = 64
  const ctx = canvas.getContext('2d')
  ctx.fillStyle = '#ffffff'
  ctx.font = '600 30px system-ui, sans-serif'
  ctx.textAlign = 'center'
  ctx.fillText('HOLD TO RESET', 128, 42)
  const texture = new THREE.CanvasTexture(canvas)
  texture.colorSpace = THREE.SRGBColorSpace
  const label = new THREE.Mesh(
    new THREE.PlaneGeometry(0.12, 0.03),
    new THREE.MeshBasicMaterial({ map: texture, transparent: true, toneMapped: false }),
  )
  label.position.set(0, 0, 0.055)
  label.quaternion.copy(FACING_ROBOT)
  group.add(label)

  const idle = new THREE.Color(0xc0392b)
  const full = new THREE.Color(0xffffff)
  return {
    group,
    setProgress: p => {
      material.color.copy(idle).lerp(full, p)
      group.scale.setScalar(1 + 0.3 * p)
    },
  }
}

// ── Calibration ──────────────────────────────────────────────────────────────
// Once per session (and again after a headset recenter): turn the world to face the way the head faces and
// put the robot's eyes under the head horizontally. Height stays the room's (the robot's eyes are at 1.24 m;
// a seated operator is lower) and the world never moves afterwards: a world that follows the head is what
// made people dizzy.

function calibrate(headPos, headQuat, worldGroup, root, eyeLocal) {
  const euler = new THREE.Euler().setFromQuaternion(headQuat, 'YXZ')
  worldGroup.position.set(0, 0, 0)
  worldGroup.rotation.set(0, euler.y, 0)
  worldGroup.updateMatrixWorld(true)
  const eye = root.localToWorld(eyeLocal.clone())
  worldGroup.position.x += headPos.x - eye.x
  worldGroup.position.z += headPos.z - eye.z
  worldGroup.updateMatrixWorld(true)
}

// ── Operator input ──────────────────────────────────────────────────────────

function newHandState() {
  const joints = {}
  for (const n of XR_JOINT_NAMES) joints[n] = { position: new THREE.Vector3(), quaternion: new THREE.Quaternion() }
  return {
    joints,
    pos: new OneEuroVector3(),
    quat: new QuaternionSmoother(0.02),
    fingers: new RetargetingFilter(),
    corrected: new THREE.Quaternion(),
    lastSeen: -Infinity,
  }
}

// Writes position + quaternion (w, x, y, z) of a three.js world pose in the MuJoCo frame
function writePose(p, q, out, o) {
  _v.copy(p).applyMatrix4(_inv)
  out[o] = _v.x; out[o + 1] = _v.y; out[o + 2] = _v.z
  _q.copy(_rootQinv).multiply(q)
  out[o + 3] = _q.w; out[o + 4] = _q.x; out[o + 5] = _q.y; out[o + 6] = _q.z
}

function readOperator(xrFrame, session, refSpace, world, hands, dt, input, raw) {
  const { root } = world
  root.updateWorldMatrix(true, false)
  _inv.copy(root.matrixWorld).invert()
  root.getWorldQuaternion(_rootQinv).invert()
  input.fill(0)
  raw.fill(0)
  writePose(_headPos, _headQuat, raw, 0)
  const now = performance.now() / 1000

  const seen = [false, false]
  for (const source of session.inputSources) {
    if (!source.hand) continue
    const s = source.handedness === 'left' ? 0 : source.handedness === 'right' ? 1 : -1
    if (s < 0) continue
    const h = hands[s]
    seen[s] = true
    const joints = {}
    XR_JOINT_NAMES.forEach((name, i) => {
      const space = source.hand.get(name)
      const pose = space && xrFrame.getJointPose(space, refSpace)
      if (!pose) return
      const j = h.joints[name]
      const { position: p, orientation: q } = pose.transform
      j.position.set(p.x, p.y, p.z)
      j.quaternion.set(q.x, q.y, q.z, q.w)
      joints[name] = j
      writePose(j.position, j.quaternion, raw, 7 + s * RAW_HAND + 7 * i)
    })
    const wrist = joints.wrist
    if (!wrist) continue

    // Re-acquired after a dropout: start the filters fresh instead of sweeping from the old pose
    if (now - h.lastSeen > HAND_DROPOUT_S) { h.pos.reset(); h.quat.reset(); h.fingers.reset() }
    h.lastSeen = now

    const o = s * HAND_INPUT
    input[o] = 1
    const pos = h.pos.update(wrist.position, dt)
    h.corrected.copy(wrist.quaternion).multiply(CORRECTION[s])
    writePose(pos, h.quat.update(h.corrected, dt), input, o + 1)
    const f = h.fingers.update(retargetHand(joints), dt)
    input.set([f.thumb.abduction, f.thumb.curl[0], f.thumb.curl[1], f.index.curl[0], f.index.curl[1],
      f.middle.curl[0], f.middle.curl[1]], o + 8)

    // Ghost: fade in with the gap between where the operator's wrist is and where the robot palm got to
    world.wristBodies[s].localToWorld(_palm.copy(PALM_OFFSET[s]))
    const gap = _palm.distanceTo(pos)
    world.ghosts[s].update(joints, Math.min(1, Math.max(0, (gap - GHOST_SHOW_AT) / (GHOST_FULL_AT - GHOST_SHOW_AT))))
  }
  for (let s = 0; s < 2; s++) if (!seen[s]) world.ghosts[s].group.visible = false
}

function setStatusText(text) {
  const el = document.getElementById('sim-status')
  if (el) el.textContent = text
}
