// Soft parcels: XPBD (extended position-based dynamics) lattices stepped in lockstep with MuJoCo. MuJoCo's own
// deformables (flex) cost too much on the headset; this is a few hundred constraints per parcel, pure JS, and
// deterministic (float64, fixed iteration order, sqrt only), so a recorded episode replays it bit for bit like
// the rigid state.
//
// Coupling with MuJoCo is through its compiled geoms: every colliding geom (robot hull meshes as fitted 26-plane polytopes,
// scene boxes, cylinders, planes, spheres, capsules, the rigid parcels) is a kinematic collider for the
// particles and surface samples, with Coulomb friction against the surface's actual velocity (a turning roller
// carries a bag), and a reaction goes back to MuJoCo bodies through xfrc_applied (see REACTION_SCALE: a
// documented fraction of the contact impulse, so a hand feels part of the bag's weight and a bag pushes a
// rigid parcel). Soft-soft contact keeps each bag's samples out of the other bag's bounding box. A weak shape
// match toward the undeformed lattice lets a crushed bag spring back instead of staying inverted.
//
// The solver constants below are part of what a recorded episode depends on (TaskSim writes them into the
// header as soft_solver); the coupling is tuned for a 2 ms MuJoCo step, which the conveyor task pins.

const GRAVITY = -9.81
const PARTICLE_RADIUS = 0.004   // contact offset: a bag's skin
const SUBSTEPS = 2              // per MuJoCo step (1 ms at the 2 ms step the soft tasks pin)
const ITERATIONS = 2            // constraint passes per substep
const LAYER_GAP = 0.5           // a particle column's layers keep at least this fraction of their rest spacing (no inverted lattice)
const PROXY_MARGIN = 0.001      // m: the proxy box's faces sit this far outside the bag's surface samples
const PROXY_TAU = 0.05          // s: low-pass on the proxy box's pose and size
// Tunable coupling constants, mutable so headless sweeps can A/B them; recorded in every header via SOFT_SOLVER()
export const TUNING = {
  reactionScale: 0.5,   // see the note below (0.15 before the inelastic velocity pass; swept over seeds since)
  normalDamping: 1.0,   // N s/m per contact sample on the relative normal approach speed (a box resting on a bag buzzes without it)
  shapeMatch: 0.02,     // per substep: pull toward the best-fit rigid placement of the rest lattice
  forceTau: 0,          // s: low-pass on the reaction force (0 = none)
}
const MAX_LIFT = 0.02           // a particle is pushed out of a collider by at most this per substep (tunnelling guard)
const SOFT_PRIORITY = 2         // like the rigid objects: a higher-priority geom's friction wins (the chute), a tie takes the larger
// Reaction on MuJoCo bodies: the normal part is a penalty on how far a contact pushed the sample this substep,
// with stiffness TUNING.reactionScale x (sample mass / substep^2): that is REACTION_SCALE times the impulse the
// sample actually received, so the transferred force is consistent across bag mass and timestep (at rest it
// is REACTION_SCALE x the resting weight on the body). The full impulse is an explicit, stiff exchange that
// bounces a 400 g box and launches it (measured, with and without low-pass filtering); 0.15 is stable for the
// bodies in these scenes. The tangential (friction) part is the sample's impulse as a force over the step.
/** Everything in this file a replay depends on, for the episode header. */
export const SOFT_SOLVER = () => ({ substeps: SUBSTEPS, iterations: ITERATIONS, particleRadius: PARTICLE_RADIUS, maxLift: MAX_LIFT, reactionScale: TUNING.reactionScale, shapeMatch: TUNING.shapeMatch, forceTau: TUNING.forceTau, proxyMargin: PROXY_MARGIN, proxyTau: PROXY_TAU, layerGap: LAYER_GAP, normalDamping: TUNING.normalDamping, surfaceSamples: 4 })
export const DEBUG = { on: false, contacts: 0, lift: 0, slip: 0, fric: 0, surf: [0, 0, 0], mu: 0, track: -1, log: [] }
// Collision samples: the particles plus, on every surface triangle, its centroid and three edge midpoints.
// Particles alone (radius 4 mm, spacing 25 mm) let a 17 mm finger pass between them and impale the bag.
const SURFACE_SAMPLES = [[1 / 3, 1 / 3, 1 / 3], [0.5, 0.5, 0], [0, 0.5, 0.5], [0.5, 0, 0.5]]
const NEAR = 0.03               // a triangle's samples are only tested against a collider one of its vertices is this close to
export { NEAR }
const PLANE = 0, SPHERE = 2, CAPSULE = 3, CYLINDER = 5, BOX = 6, MESH = 7
const IDENTITY = Float64Array.from([1, 0, 0, 0, 1, 0, 0, 0, 1])
// Hull meshes are collided as a 26-plane polytope (axes, face diagonals, corner diagonals) fitted to the hull
// vertices: a box fitted by PCA left up to 2.6 cm of air around the palm, enough to trap a bag between palm
// and thumb. The signed distance is the largest plane distance: exact inside and near faces, a slight
// underestimate near edges and corners.
const DOP_DIRS = (() => {
  const out = []
  for (let x = -1; x <= 1; x++) for (let y = -1; y <= 1; y++) for (let z = -1; z <= 1; z++) {
    if (!x && !y && !z) continue
    const l = Math.sqrt(x * x + y * y + z * z)
    out.push(x / l, y / l, z / l)
  }
  return Float64Array.from(out)
})()
const NDOP = DOP_DIRS.length / 3

/** Lattice of (nx x ny x nz) cells over a box of half extents `half`: particles, distance and volume constraints,
 * surface triangles (the top face last, with UVs, so a label can be drawn on it). */
export function softLattice({ half, cells }) {
  const [nx, ny, nz] = cells
  const [hx, hy, hz] = half
  const id = (i, j, k) => (i * (ny + 1) + j) * (nz + 1) + k
  const n = (nx + 1) * (ny + 1) * (nz + 1)
  const rest = new Float64Array(3 * n)
  for (let i = 0; i <= nx; i++) for (let j = 0; j <= ny; j++) for (let k = 0; k <= nz; k++) {
    const p = 3 * id(i, j, k)
    rest[p] = -hx + 2 * hx * i / nx
    rest[p + 1] = -hy + 2 * hy * j / ny
    rest[p + 2] = -hz + 2 * hz * k / nz
  }
  // five tetrahedra per cell, mirrored on odd cells so neighbours share face diagonals
  const tets = []
  for (let i = 0; i < nx; i++) for (let j = 0; j < ny; j++) for (let k = 0; k < nz; k++) {
    const c = (di, dj, dk) => id(i + di, j + dj, k + dk)
    const even = (i + j + k) % 2 === 0
    const [a000, a100, a010, a001, a110, a101, a011, a111] = even
      ? [c(0, 0, 0), c(1, 0, 0), c(0, 1, 0), c(0, 0, 1), c(1, 1, 0), c(1, 0, 1), c(0, 1, 1), c(1, 1, 1)]
      : [c(1, 0, 0), c(0, 0, 0), c(1, 1, 0), c(1, 0, 1), c(0, 1, 0), c(0, 0, 1), c(1, 1, 1), c(0, 1, 1)]
    tets.push([a000, a100, a010, a001], [a110, a100, a010, a111], [a101, a100, a001, a111], [a011, a010, a001, a111], [a100, a010, a001, a111])
  }
  const vol = t => {
    const [a, b, c, d] = t.map(v => rest.subarray(3 * v, 3 * v + 3))
    const ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]], ad = [d[0] - a[0], d[1] - a[1], d[2] - a[2]]
    return ((ab[1] * ac[2] - ab[2] * ac[1]) * ad[0] + (ab[2] * ac[0] - ab[0] * ac[2]) * ad[1] + (ab[0] * ac[1] - ab[1] * ac[0]) * ad[2]) / 6
  }
  for (const t of tets) if (vol(t) < 0) [t[1], t[2]] = [t[2], t[1]]
  const restVol = Float64Array.from(tets, vol)
  // distance constraints: every tet edge once
  const seen = new Set()
  const edges = []
  for (const t of tets) for (let a = 0; a < 4; a++) for (let b = a + 1; b < 4; b++) {
    const lo = Math.min(t[a], t[b]), hi = Math.max(t[a], t[b])
    const key = lo * n + hi
    if (!seen.has(key)) { seen.add(key); edges.push(lo, hi) }
  }
  const restLen = new Float64Array(edges.length / 2)
  for (let e = 0; e < restLen.length; e++) {
    const a = 3 * edges[2 * e], b = 3 * edges[2 * e + 1]
    restLen[e] = Math.sqrt((rest[a] - rest[b]) ** 2 + (rest[a + 1] - rest[b + 1]) ** 2 + (rest[a + 2] - rest[b + 2]) ** 2)
  }
  // surface: the six faces, outward winding, top face (k = nz) last
  const tris = []
  const quad = (a, b, c, d, outward) => {
    // outward: a point on the outside; flip the winding so the normal faces it
    const pa = rest.subarray(3 * a, 3 * a + 3), pb = rest.subarray(3 * b, 3 * b + 3), pc = rest.subarray(3 * c, 3 * c + 3)
    const u = [pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]], v = [pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2]]
    const nrm = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
    const flip = nrm[0] * outward[0] + nrm[1] * outward[1] + nrm[2] * outward[2] < 0
    if (flip) tris.push(a, c, b, a, d, c)
    else tris.push(a, b, c, a, c, d)
  }
  for (let j = 0; j < ny; j++) for (let k = 0; k < nz; k++) {
    quad(id(0, j, k), id(0, j + 1, k), id(0, j + 1, k + 1), id(0, j, k + 1), [-1, 0, 0])
    quad(id(nx, j, k), id(nx, j + 1, k), id(nx, j + 1, k + 1), id(nx, j, k + 1), [1, 0, 0])
  }
  for (let i = 0; i < nx; i++) for (let k = 0; k < nz; k++) {
    quad(id(i, 0, k), id(i + 1, 0, k), id(i + 1, 0, k + 1), id(i, 0, k + 1), [0, -1, 0])
    quad(id(i, ny, k), id(i + 1, ny, k), id(i + 1, ny, k + 1), id(i, ny, k + 1), [0, 1, 0])
  }
  for (let i = 0; i < nx; i++) for (let j = 0; j < ny; j++) quad(id(i, j, 0), id(i + 1, j, 0), id(i + 1, j + 1, 0), id(i, j + 1, 0), [0, 0, -1])
  const topStart = tris.length
  for (let i = 0; i < nx; i++) for (let j = 0; j < ny; j++) quad(id(i, j, nz), id(i + 1, j, nz), id(i + 1, j + 1, nz), id(i, j + 1, nz), [0, 0, 1])
  const uv = new Float32Array(2 * n)
  for (let i = 0; i <= nx; i++) for (let j = 0; j <= ny; j++) { const p = id(i, j, nz); uv[2 * p] = i / nx; uv[2 * p + 1] = j / ny }
  const topCorners = [id(0, 0, nz), id(nx, 0, nz), id(nx, ny, nz), id(0, ny, nz)]
  return {
    n, rest, edges: Int32Array.from(edges), restLen, tets: Int32Array.from(tets.flat()), restVol,
    surface: Uint32Array.from(tris), topStart, topCount: tris.length - topStart, uv, topCorners,
  }
}

/** Rotates v by the unit quaternion q (w, x, y, z). */
function rotate(q, v, out, o = 0) {
  const [w, x, y, z] = q
  const tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0])
  out[o] = v[0] + w * tx + (y * tz - z * ty)
  out[o + 1] = v[1] + w * ty + (z * tx - x * tz)
  out[o + 2] = v[2] + w * tz + (x * ty - y * tx)
}

export class SoftBody {
  constructor(name, def) {
    this.name = name
    this.def = def
    this.lattice = softLattice(def)
    const { n } = this.lattice
    this.n = n
    this.x = new Float64Array(3 * n)
    this.xPrev = new Float64Array(3 * n)
    this.v = new Float64Array(3 * n)
    // surface samples for collision: triangle vertices (3 per sample) and barycentric weights
    const tris = this.lattice.surface
    const ns = (tris.length / 3) * SURFACE_SAMPLES.length
    this.nSamples = ns
    this.sampleIdx = new Int32Array(3 * ns)
    this.sampleW = new Float64Array(3 * ns)
    this.sampleWsum = new Float64Array(ns)
    let k = 0
    for (let t = 0; t < tris.length; t += 3) for (const w of SURFACE_SAMPLES) {
      this.sampleIdx[3 * k] = tris[t]; this.sampleIdx[3 * k + 1] = tris[t + 1]; this.sampleIdx[3 * k + 2] = tris[t + 2]
      this.sampleW[3 * k] = w[0]; this.sampleW[3 * k + 1] = w[1]; this.sampleW[3 * k + 2] = w[2]
      this.sampleWsum[k] = w[0] * w[0] + w[1] * w[1] + w[2] * w[2]
      k++
    }
    this.near = new Uint8Array(this.n) // per collider pass: particles within NEAR of it
    // contact bookkeeping for the velocity pass: the normal push each particle received this substep (which
    // must not become velocity: an inelastic contact), the last contact normal and the surface's speed along it
    this.dop = new Float64Array(NDOP) // world-space 26-plane bound of the particles, for bag-bag contact
    this.R = Float64Array.from([1, 0, 0, 0, 1, 0, 0, 0, 1]) // last best-fit rotation of the rest lattice (shape matching)
    this.moved = false
    this.pressed = false // a moving body (hand, box) touched this bag during the last step
    this.push = new Float64Array(3 * this.n)
    this.contactN = new Float64Array(3 * this.n)
    this.contactVn = new Float64Array(this.n)
    this.inContact = new Uint8Array(this.n)
    this.shape = { A: new Float64Array(9), S2: new Float64Array(9), V: new Float64Array(9), R: new Float64Array(9) }
    // vertical particle columns (consecutive layers), for the ordering constraint
    const [nx, ny, nz] = def.cells
    this.columns = new Int32Array((nx + 1) * (ny + 1) * (nz + 1))
    let col = 0
    for (let i = 0; i <= nx; i++) for (let j = 0; j <= ny; j++) for (let kk = 0; kk <= nz; kk++) this.columns[col++] = (i * (ny + 1) + j) * (nz + 1) + kk
    this.layers = nz + 1
    this.layerSpacing = 2 * def.half[2] / nz
    this.lambdaEdge = new Float64Array(this.lattice.restLen.length)
    this.lambdaVol = new Float64Array(this.lattice.restVol.length)
    this.aabb = new Float64Array(6)
    this.active = false // frozen (parked) bodies are skipped entirely
    this.setPhysics({})
  }

  /** mass (kg), friction, edge/volume compliance (m/N; larger = softer), damping (1/s) */
  setPhysics({ mass = this.def.mass ?? 0.3, friction = this.def.friction ?? 0.6, edgeCompliance = this.def.edgeCompliance ?? 2e-3, volumeCompliance = this.def.volumeCompliance ?? 1e-7, damping = this.def.damping ?? 1 } = {}) {
    this.params = { mass, friction, edgeCompliance, volumeCompliance, damping }
    this.invMass = this.n / mass
  }

  /** Places the body at rest (undeformed) with its centre at pos and orientation quat (w, x, y, z). */
  place(pos, quat = [1, 0, 0, 0], active = true) {
    const { rest } = this.lattice
    const tmp = [0, 0, 0]
    for (let i = 0; i < this.n; i++) {
      rotate(quat, rest.subarray(3 * i, 3 * i + 3), tmp)
      this.x[3 * i] = pos[0] + tmp[0]; this.x[3 * i + 1] = pos[1] + tmp[1]; this.x[3 * i + 2] = pos[2] + tmp[2]
    }
    this.xPrev.set(this.x)
    this.v.fill(0)
    this.active = active
    this.updateAabb()
    // the placed orientation, for the proxy and the layer ordering until shape matching refits it
    const [w, qx, qy, qz] = quat
    this.R.set([1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - w * qz), 2 * (qx * qz + w * qy),
      2 * (qx * qy + w * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - w * qx),
      2 * (qx * qz - w * qy), 2 * (qy * qz + w * qx), 1 - 2 * (qx * qx + qy * qy)])
    this.moved = true // the proxy jumps to the new pose instead of sliding there
  }

  /** Fits the 26-plane bound to the current particles (world frame). */
  updateDop() {
    const { x, n, dop } = this
    dop.fill(-Infinity)
    for (let i = 0; i < n; i++) {
      const px = x[3 * i], py = x[3 * i + 1], pz = x[3 * i + 2]
      for (let k = 0; k < NDOP; k++) {
        const d = DOP_DIRS[3 * k] * px + DOP_DIRS[3 * k + 1] * py + DOP_DIRS[3 * k + 2] * pz
        if (d > dop[k]) dop[k] = d
      }
    }
  }

  updateAabb() {
    const b = this.aabb
    b[0] = b[1] = b[2] = Infinity; b[3] = b[4] = b[5] = -Infinity
    const { x } = this
    for (let i = 0; i < this.n; i++) {
      const p = 3 * i
      if (x[p] < b[0]) b[0] = x[p]; if (x[p] > b[3]) b[3] = x[p]
      if (x[p + 1] < b[1]) b[1] = x[p + 1]; if (x[p + 1] > b[4]) b[4] = x[p + 1]
      if (x[p + 2] < b[2]) b[2] = x[p + 2]; if (x[p + 2] > b[5]) b[5] = x[p + 2]
    }
  }

  centroid() {
    const { x } = this
    let cx = 0, cy = 0, cz = 0
    for (let i = 0; i < this.n; i++) { cx += x[3 * i]; cy += x[3 * i + 1]; cz += x[3 * i + 2] }
    return [cx / this.n, cy / this.n, cz / this.n]
  }

  /** z component of the top face's outward normal: 1 label up, -1 label down. */
  up() {
    // mean (area-weighted) normal of the top face's triangles
    const { x } = this
    const { surface, topStart } = this.lattice
    let nx = 0, ny = 0, nz = 0
    for (let t = topStart; t < surface.length; t += 3) {
      const a = 3 * surface[t], b = 3 * surface[t + 1], c = 3 * surface[t + 2]
      const ux = x[b] - x[a], uy = x[b + 1] - x[a + 1], uz = x[b + 2] - x[a + 2]
      const vx = x[c] - x[a], vy = x[c + 1] - x[a + 1], vz = x[c + 2] - x[a + 2]
      nx += uy * vz - uz * vy; ny += uz * vx - ux * vz; nz += ux * vy - uy * vx
    }
    const len = Math.sqrt(nx * nx + ny * ny + nz * nz)
    return len > 0 ? nz / len : 0
  }

  /** Fraction of the rest volume the lattice currently encloses, and how many tetrahedra are inverted. */
  volumeState() {
    const { x } = this
    const { tets, restVol } = this.lattice
    let vol = 0, inverted = 0, rest = 0
    for (let t = 0; t < restVol.length; t++) {
      const i0 = 3 * tets[4 * t], i1 = 3 * tets[4 * t + 1], i2 = 3 * tets[4 * t + 2], i3 = 3 * tets[4 * t + 3]
      const e1x = x[i1] - x[i0], e1y = x[i1 + 1] - x[i0 + 1], e1z = x[i1 + 2] - x[i0 + 2]
      const e2x = x[i2] - x[i0], e2y = x[i2 + 1] - x[i0 + 1], e2z = x[i2 + 2] - x[i0 + 2]
      const e3x = x[i3] - x[i0], e3y = x[i3 + 1] - x[i0 + 1], e3z = x[i3 + 2] - x[i0 + 2]
      const v = ((e2y * e3z - e2z * e3y) * e1x + (e2z * e3x - e2x * e3z) * e1y + (e2x * e3y - e2y * e3x) * e1z) / 6
      vol += v; rest += restVol[t]
      if (v < 0) inverted++
    }
    return { volume: vol / rest, inverted }
  }

  /** Shape matching: the best-fit rigid placement of the rest lattice (Muller et al.), pulled toward weakly. */
  shapeMatch(k) {
    const { x, n, lattice: { rest }, shape: { A, S2, V, R } } = this
    let cx = 0, cy = 0, cz = 0
    for (let i = 0; i < n; i++) { cx += x[3 * i]; cy += x[3 * i + 1]; cz += x[3 * i + 2] }
    cx /= n; cy /= n; cz /= n
    A.fill(0)
    for (let i = 0; i < n; i++) {
      const px = x[3 * i] - cx, py = x[3 * i + 1] - cy, pz = x[3 * i + 2] - cz // rest is centred at the origin
      const qx = rest[3 * i], qy = rest[3 * i + 1], qz = rest[3 * i + 2]
      A[0] += px * qx; A[1] += px * qy; A[2] += px * qz
      A[3] += py * qx; A[4] += py * qy; A[5] += py * qz
      A[6] += pz * qx; A[7] += pz * qy; A[8] += pz * qz
    }
    // R = A (A^T A)^-1/2 by eigen decomposition of A^T A
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) { let t = 0; for (let k2 = 0; k2 < 3; k2++) t += A[3 * k2 + r] * A[3 * k2 + c]; S2[3 * r + c] = t }
    eigen3(S2, V)
    const inv = [0, 1, 2].map(a => { let lam = 0; for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) lam += V[3 * r + a] * S2[3 * r + c] * V[3 * c + a]; return lam > 1e-18 ? 1 / Math.sqrt(lam) : 0 })
    // S^-1 = V diag(inv) V^T, R = A S^-1
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) {
      let t = 0
      for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) t += A[3 * r + a] * V[3 * a + b] * inv[b] * V[3 * c + b]
      R[3 * r + c] = t
    }
    const det = R[0] * (R[4] * R[8] - R[5] * R[7]) - R[1] * (R[3] * R[8] - R[5] * R[6]) + R[2] * (R[3] * R[7] - R[4] * R[6])
    if (!(det > 0.5)) return // degenerate (flat or reflected): leave it to the constraints
    this.R.set(R)
    for (let i = 0; i < n; i++) {
      const qx = rest[3 * i], qy = rest[3 * i + 1], qz = rest[3 * i + 2]
      const gx = R[0] * qx + R[1] * qy + R[2] * qz + cx, gy = R[3] * qx + R[4] * qy + R[5] * qz + cy, gz = R[6] * qx + R[7] * qy + R[8] * qz + cz
      x[3 * i] += (gx - x[3 * i]) * k; x[3 * i + 1] += (gy - x[3 * i + 1]) * k; x[3 * i + 2] += (gz - x[3 * i + 2]) * k
    }
    // Layer ordering along the lattice's own up axis: a column's layers never pass through each other, so a
    // crushing finger cannot leave the lattice inverted (distance constraints alone are satisfied by the
    // mirrored configuration too)
    const ux = R[2], uy = R[5], uz = R[8]
    const gap = LAYER_GAP * this.layerSpacing
    const { columns, layers } = this
    for (let c0 = 0; c0 < columns.length; c0 += layers) {
      for (let l = 1; l < layers; l++) {
        const a = 3 * columns[c0 + l - 1], b = 3 * columns[c0 + l]
        const dz = (x[b] - x[a]) * ux + (x[b + 1] - x[a + 1]) * uy + (x[b + 2] - x[a + 2]) * uz
        if (dz < gap) {
          const h = (gap - dz) / 2
          x[a] -= ux * h; x[a + 1] -= uy * h; x[a + 2] -= uz * h
          x[b] += ux * h; x[b + 1] += uy * h; x[b + 2] += uz * h
        }
      }
    }
  }

  /** Mean particle speed (m/s). */
  speed() {
    const { v } = this
    let s = 0
    for (let i = 0; i < this.n; i++) s += Math.sqrt(v[3 * i] ** 2 + v[3 * i + 1] ** 2 + v[3 * i + 2] ** 2)
    return s / this.n
  }

  finite() {
    for (let i = 0; i < this.x.length; i++) if (!Number.isFinite(this.x[i])) return false
    return true
  }
}

// Point-in-geom-frame signed distance and outward normal for each MuJoCo geom type (and the box fitted to a
// hull mesh). Writes [distance, nx, ny, nz] (local frame) into out.
function localDistance(c, p, out) {
  const [px, py, pz] = p
  switch (c.type) {
    case PLANE: out[0] = pz; out[1] = 0; out[2] = 0; out[3] = 1; return
    case SPHERE: {
      const r = Math.sqrt(px * px + py * py + pz * pz) || 1e-12
      out[0] = r - c.size[0]; out[1] = px / r; out[2] = py / r; out[3] = pz / r; return
    }
    case CAPSULE: {
      const z = Math.max(-c.size[1], Math.min(c.size[1], pz))
      const dz = pz - z
      const r = Math.sqrt(px * px + py * py + dz * dz) || 1e-12
      out[0] = r - c.size[0]; out[1] = px / r; out[2] = py / r; out[3] = dz / r; return
    }
    case CYLINDER: {
      const rad = Math.sqrt(px * px + py * py)
      const dr = rad - c.size[0], dz = Math.abs(pz) - c.size[1]
      if (dr > 0 && dz > 0) { // edge region
        const d = Math.sqrt(dr * dr + dz * dz)
        out[0] = d; out[1] = px / rad * dr / d; out[2] = py / rad * dr / d; out[3] = Math.sign(pz) * dz / d
      } else if (dr > dz) { // nearest to the side
        out[0] = dr; out[1] = rad > 0 ? px / rad : 1; out[2] = rad > 0 ? py / rad : 0; out[3] = 0
      } else { out[0] = dz; out[1] = 0; out[2] = 0; out[3] = pz >= 0 ? 1 : -1 }
      return
    }
    case MESH: { // fitted polytope: max over its planes of (n . p - d)
      const planes = c.planes
      let best = -Infinity, bi = 0
      for (let i = 0; i < NDOP; i++) {
        const v = DOP_DIRS[3 * i] * px + DOP_DIRS[3 * i + 1] * py + DOP_DIRS[3 * i + 2] * pz - planes[i]
        if (v > best) { best = v; bi = i }
      }
      out[0] = best; out[1] = DOP_DIRS[3 * bi]; out[2] = DOP_DIRS[3 * bi + 1]; out[3] = DOP_DIRS[3 * bi + 2]
      return
    }
    default: { // BOX
      const half = c.size
      const cx = 0, cy = 0, cz = 0
      const qx = Math.abs(px - cx) - half[0], qy = Math.abs(py - cy) - half[1], qz = Math.abs(pz - cz) - half[2]
      const sx = px - cx >= 0 ? 1 : -1, sy = py - cy >= 0 ? 1 : -1, sz = pz - cz >= 0 ? 1 : -1
      if (qx > 0 || qy > 0 || qz > 0) {
        const ox = Math.max(qx, 0), oy = Math.max(qy, 0), oz = Math.max(qz, 0)
        const d = Math.sqrt(ox * ox + oy * oy + oz * oz)
        out[0] = d; out[1] = sx * ox / d; out[2] = sy * oy / d; out[3] = sz * oz / d
      } else if (qx >= qy && qx >= qz) { out[0] = qx; out[1] = sx; out[2] = 0; out[3] = 0 }
      else if (qy >= qz) { out[0] = qy; out[1] = 0; out[2] = sy; out[3] = 0 }
      else { out[0] = qz; out[1] = 0; out[2] = 0; out[3] = sz }
    }
  }
}

// Eigenvectors of a symmetric 3x3 matrix (cyclic Jacobi), columns of out (row-major)
function eigen3(a, out) {
  const m = Float64Array.from(a)
  out.set([1, 0, 0, 0, 1, 0, 0, 0, 1])
  for (let sweep = 0; sweep < 20; sweep++) {
    let off = 0
    for (let p = 0; p < 3; p++) for (let q = p + 1; q < 3; q++) off += m[3 * p + q] ** 2
    if (off < 1e-20) break
    for (let p = 0; p < 3; p++) for (let q = p + 1; q < 3; q++) {
      if (Math.abs(m[3 * p + q]) < 1e-30) continue
      const theta = (m[3 * q + q] - m[3 * p + p]) / (2 * m[3 * p + q])
      const t = (theta >= 0 ? 1 : -1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1))
      const c = 1 / Math.sqrt(t * t + 1), s = t * c
      for (let k = 0; k < 3; k++) {
        const mkp = m[3 * k + p], mkq = m[3 * k + q]
        m[3 * k + p] = c * mkp - s * mkq; m[3 * k + q] = s * mkp + c * mkq
      }
      for (let k = 0; k < 3; k++) {
        const mpk = m[3 * p + k], mqk = m[3 * q + k]
        m[3 * p + k] = c * mpk - s * mqk; m[3 * q + k] = s * mpk + c * mqk
      }
      for (let k = 0; k < 3; k++) {
        const okp = out[3 * k + p], okq = out[3 * k + q]
        out[3 * k + p] = c * okp - s * okq; out[3 * k + q] = s * okp + c * okq
      }
    }
  }
}

export class SoftWorld {
  /**
   * @param defs   { [name]: { half, cells, mass?, friction?, edgeCompliance?, volumeCompliance?, damping? } }
   * @param info   { handOfBody: Int8Array (0/1/-1 per MuJoCo body), belts: geom-name prefixes of roller belts }
   *
   * A roller belt (cylinders named with a `belts` prefix) is collided as one flat moving surface: particles are
   * larger than the gaps between rollers and would wedge into them, where the two rollers' friction cancels.
   */
  constructor(mj, m, defs, { handOfBody, belts = [], proxies = {} } = {}) {
    this.mj = mj
    this.m = m
    this.belts = belts
    // mocap proxy boxes (see the scene XML): { bodyName } per soft body, resolved to mocap and geom ids
    this.proxyGeoms = new Set()
    this.proxies = Object.fromEntries(Object.entries(proxies).map(([name, body]) => {
      const b = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY.value, body)
      if (b < 0 || m.body_mocapid[b] < 0) throw new Error(`soft: ${body} is not a mocap body`)
      const geom = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM.value, body)
      if (geom < 0) throw new Error(`soft: proxy ${body} needs a geom of the same name`)
      this.proxyGeoms.add(geom)
      return [name, { mocap: m.body_mocapid[b], geom }]
    }))
    this.bodies = Object.entries(defs).map(([name, def]) => new SoftBody(name, def))
    this.byName = Object.fromEntries(this.bodies.map(b => [b.name, b]))
    this.total = this.bodies.reduce((s, b) => s + b.n, 0)
    this.handOfBody = handOfBody ?? new Int8Array(m.nbody).fill(-1)
    this.touching = new Uint8Array(2)
    this.colliders = this.buildColliders()
    this.force = new Float64Array(6 * m.nbody)   // reaction this step, written to xfrc_applied (filtered if forceTau)
    this.forceFilt = new Float64Array(6 * m.nbody)
    this.dtStep = m.opt.timestep
    this.vel = new Float64Array(6)
    this.pt = new Float64Array(3)
    this.prev = new Float64Array(3)
    this.out = new Float64Array(4)
    this.local = [0, 0, 0]
    this.worldN = [0, 0, 0]
    this.maxPen = 0
  }

  // Every geom that collides, as a kinematic collider; hull meshes as the box fitted to their vertices
  buildColliders() {
    const { m } = this
    const list = []
    const rollers = this.belts.map(() => [])
    for (let g = 0; g < m.ngeom; g++) {
      if (!m.geom_contype[g] && !m.geom_conaffinity[g]) continue
      if (this.proxyGeoms.has(g)) continue // a bag's own proxy is not an obstacle to it
      const type = m.geom_type[g]
      const gname = this.mj.mj_id2name(m, this.mj.mjtObj.mjOBJ_GEOM.value, g) ?? ''
      const belt = this.belts.findIndex(prefix => gname.startsWith(prefix))
      if (belt >= 0 && type === CYLINDER) { rollers[belt].push(g); continue }
      // dynamic: some joint moves this body (a static fixture gets no reaction force and no velocity read)
      let dynamic = false
      for (let b = m.geom_bodyid[g]; b > 0; b = m.body_parentid[b]) if (m.body_dofnum[b] > 0) { dynamic = true; break }
      const c = {
        geom: g, body: m.geom_bodyid[g], type, size: Array.from(m.geom_size.subarray(3 * g, 3 * g + 3)),
        priority: m.geom_priority[g], dynamic, // friction is read at contact time: it is randomized per episode
        radius: 0, // bounding radius about the geom origin, for the broad phase
      }
      if (type === MESH) {
        const mesh = m.geom_dataid[g]
        const va = m.mesh_vertadr[mesh], vn = m.mesh_vertnum[mesh]
        const vert = m.mesh_vert.subarray(3 * va, 3 * (va + vn))
        const planes = new Float64Array(NDOP).fill(-Infinity)
        let rad2 = 0
        for (let i = 0; i < vn; i++) {
          const x = vert[3 * i], y = vert[3 * i + 1], z = vert[3 * i + 2]
          rad2 = Math.max(rad2, x * x + y * y + z * z)
          for (let k = 0; k < NDOP; k++) {
            const d = DOP_DIRS[3 * k] * x + DOP_DIRS[3 * k + 1] * y + DOP_DIRS[3 * k + 2] * z
            if (d > planes[k]) planes[k] = d
          }
        }
        c.planes = planes
        c.radius = Math.sqrt(rad2)
      } else if (type === PLANE) c.radius = Infinity
      else if (type === BOX) c.radius = Math.sqrt(c.size[0] ** 2 + c.size[1] ** 2 + c.size[2] ** 2)
      else if (type === CYLINDER || type === CAPSULE) c.radius = Math.sqrt(c.size[0] ** 2 + c.size[1] ** 2) + (type === CAPSULE ? c.size[0] : 0)
      else c.radius = c.size[0]
      list.push(c)
    }
    // static fixtures last: when a hand presses a bag into the plate, the plate's push is applied after the
    // hand's, so no sample is driven through the plate (a 20 mm sheet) and ejected underneath
    list.sort((a, b) => Number(a.dynamic === false) - Number(b.dynamic === false))
    for (const geoms of rollers) {
      if (!geoms.length) continue
      // the belt surface: a static box spanning the rollers (axis along world x), top at the rollers' top,
      // moving at the first roller's rim speed
      const g0 = geoms[0], b0 = m.geom_bodyid[g0]
      if (geoms.some(g => m.body_parentid[m.geom_bodyid[g]] !== 0)) throw new Error('soft: belt rollers must be direct children of the world body')
      const R = m.geom_size[3 * g0], halfLen = m.geom_size[3 * g0 + 1]
      const ys = geoms.map(g => m.body_pos[3 * m.geom_bodyid[g] + 1])
      const yc = (Math.min(...ys) + Math.max(...ys)) / 2, halfY = (Math.max(...ys) - Math.min(...ys)) / 2 + R
      list.push({
        geom: g0, body: b0, type: BOX, size: [halfLen, halfY, R], priority: m.geom_priority[g0],
        dynamic: false, radius: Math.sqrt(halfLen * halfLen + halfY * halfY + R * R),
        virtual: { pos: [m.body_pos[3 * b0], yc, m.body_pos[3 * b0 + 2]], spinBody: b0, spinRadius: R },
      })
    }
    return list
  }

  setPhysics(params = {}) {
    for (const b of this.bodies) b.setPhysics(params[b.name] ?? {})
  }

  /** Writes every body's particle positions, in order, into out (Float32Array or Float64Array). */
  gather(out) {
    let o = 0
    for (const b of this.bodies) { out.set(b.x, o); o += 3 * b.n }
    return out
  }

  /** Restores every body's particle positions from a flat array (velocities zeroed). */
  scatter(positions) {
    let o = 0
    for (const b of this.bodies) { b.x.set(positions.subarray ? positions.subarray(o, o + 3 * b.n) : positions.slice(o, o + 3 * b.n)); b.xPrev.set(b.x); b.v.fill(0); b.updateAabb(); o += 3 * b.n }
  }

  finite() { return this.bodies.every(b => b.finite()) }

  /**
   * Advances every active body by one MuJoCo step (d.opt.timestep via m), colliding against d's current geom
   * poses, and leaves the reaction forces in d.xfrc_applied for the coming mj_step. Call right before mj_step.
   */
  step(d) {
    const { m } = this
    this.dtStep = m.opt.timestep // one embind read per step, not per contact
    const dt = this.dtStep / SUBSTEPS
    this.force.fill(0)
    this.touching.fill(0)
    this.maxPen = 0
    const active = this.bodies.filter(b => b.active)
    for (const b of active) b.pressed = false
    if (active.length) {
      for (let s = 0; s < SUBSTEPS; s++) {
        for (const b of active) this.substep(b, d, dt)
        if (active.length > 1) this.softContacts(active)
      }
    }
    this.updateProxies(d)
    if (TUNING.forceTau > 0) {
      const alpha = Math.min(1, this.dtStep / TUNING.forceTau)
      for (let i = 0; i < this.force.length; i++) this.forceFilt[i] += (this.force[i] - this.forceFilt[i]) * alpha
      d.xfrc_applied.set(this.forceFilt)
    } else d.xfrc_applied.set(this.force)
  }

  // Each bag's proxy box: centred on the particles' extents along the bag's own axes (last shape-match
  // rotation), half extents the particle extents plus the skin and a 1 mm margin (a box resting on the proxy
  // then sits just above the surface samples, so it does not keep nudging the particles), pose and size
  // low-passed over PROXY_TAU so particle jitter does not shake what rests on it; a parked bag takes its proxy
  // along (far from the scene) at once
  updateProxies(d) {
    const { m } = this
    const alpha = Math.min(1, this.dtStep / PROXY_TAU)
    for (const b of this.bodies) {
      const p = this.proxies[b.name]
      if (!p) continue
      const { x, n, R } = b
      const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity]
      for (let i = 0; i < n; i++) {
        const px = x[3 * i], py = x[3 * i + 1], pz = x[3 * i + 2]
        for (let a = 0; a < 3; a++) { // projection on column a of R
          const v = R[a] * px + R[3 + a] * py + R[6 + a] * pz
          if (v < lo[a]) lo[a] = v
          if (v > hi[a]) hi[a] = v
        }
      }
      const mid = [0, 1, 2].map(a => (lo[a] + hi[a]) / 2)
      const half = [0, 1, 2].map(a => (hi[a] - lo[a]) / 2 + PARTICLE_RADIUS + PROXY_MARGIN)
      // centre back in the world frame: R * mid
      const cx = R[0] * mid[0] + R[1] * mid[1] + R[2] * mid[2], cy = R[3] * mid[0] + R[4] * mid[1] + R[5] * mid[2], cz = R[6] * mid[0] + R[7] * mid[1] + R[8] * mid[2]
      const k = b.moved ? 1 : alpha // a teleport (place) moves the proxy at once
      b.moved = false
      const mp = d.mocap_pos, o = 3 * p.mocap
      mp[o] += (cx - mp[o]) * k; mp[o + 1] += (cy - mp[o + 1]) * k; mp[o + 2] += (cz - mp[o + 2]) * k
      for (let a = 0; a < 3; a++) half[a] = m.geom_size[3 * p.geom + a] + (half[a] - m.geom_size[3 * p.geom + a]) * k
      // quaternion (w, x, y, z) of R
      const tr = R[0] + R[4] + R[8]
      let qw, qx, qy, qz
      if (tr > 0) { const s = Math.sqrt(tr + 1) * 2; qw = 0.25 * s; qx = (R[7] - R[5]) / s; qy = (R[2] - R[6]) / s; qz = (R[3] - R[1]) / s }
      else if (R[0] > R[4] && R[0] > R[8]) { const s = Math.sqrt(1 + R[0] - R[4] - R[8]) * 2; qw = (R[7] - R[5]) / s; qx = 0.25 * s; qy = (R[1] + R[3]) / s; qz = (R[2] + R[6]) / s }
      else if (R[4] > R[8]) { const s = Math.sqrt(1 + R[4] - R[0] - R[8]) * 2; qw = (R[2] - R[6]) / s; qx = (R[1] + R[3]) / s; qy = 0.25 * s; qz = (R[5] + R[7]) / s }
      else { const s = Math.sqrt(1 + R[8] - R[0] - R[4]) * 2; qw = (R[3] - R[1]) / s; qx = (R[2] + R[6]) / s; qy = (R[5] + R[7]) / s; qz = 0.25 * s }
      const mq = d.mocap_quat, q = 4 * p.mocap
      if (mq[q] * qw + mq[q + 1] * qx + mq[q + 2] * qy + mq[q + 3] * qz < 0) { qw = -qw; qx = -qx; qy = -qy; qz = -qz } // same hemisphere
      mq[q] += (qw - mq[q]) * k; mq[q + 1] += (qx - mq[q + 1]) * k; mq[q + 2] += (qy - mq[q + 2]) * k; mq[q + 3] += (qz - mq[q + 3]) * k
      const qn = Math.sqrt(mq[q] * mq[q] + mq[q + 1] * mq[q + 1] + mq[q + 2] * mq[q + 2] + mq[q + 3] * mq[q + 3]) || 1
      mq[q] /= qn; mq[q + 1] /= qn; mq[q + 2] /= qn; mq[q + 3] /= qn
      m.geom_size[3 * p.geom] = half[0]; m.geom_size[3 * p.geom + 1] = half[1]; m.geom_size[3 * p.geom + 2] = half[2]
      m.geom_rbound[p.geom] = Math.sqrt(half[0] * half[0] + half[1] * half[1] + half[2] * half[2])
      // MuJoCo's broad phase uses the compile-time bounding box: keep it in step with the size
      const ab = 6 * p.geom
      m.geom_aabb[ab] = 0; m.geom_aabb[ab + 1] = 0; m.geom_aabb[ab + 2] = 0
      m.geom_aabb[ab + 3] = half[0]; m.geom_aabb[ab + 4] = half[1]; m.geom_aabb[ab + 5] = half[2]
    }
  }

  substep(b, d, dt) {
    const { x, xPrev, v, n, invMass, lattice, params } = b
    b.push.fill(0)
    b.inContact.fill(0)
    const g = GRAVITY * dt
    // predict
    for (let i = 0; i < n; i++) {
      const p = 3 * i
      xPrev[p] = x[p]; xPrev[p + 1] = x[p + 1]; xPrev[p + 2] = x[p + 2]
      v[p + 2] += g
      x[p] += v[p] * dt; x[p + 1] += v[p + 1] * dt; x[p + 2] += v[p + 2] * dt
    }
    // constraints (XPBD, lambdas reset each substep)
    b.lambdaEdge.fill(0)
    b.lambdaVol.fill(0)
    const alphaE = params.edgeCompliance / (dt * dt), alphaV = params.volumeCompliance / (dt * dt)
    const { edges, restLen, tets, restVol } = lattice
    for (let it = 0; it < ITERATIONS; it++) {
      for (let e = 0; e < restLen.length; e++) {
        const a = 3 * edges[2 * e], c = 3 * edges[2 * e + 1]
        const dx = x[a] - x[c], dy = x[a + 1] - x[c + 1], dz = x[a + 2] - x[c + 2]
        const len = Math.sqrt(dx * dx + dy * dy + dz * dz)
        if (len < 1e-9) continue
        const C = len - restLen[e]
        const dl = (-C - alphaE * b.lambdaEdge[e]) / (2 * invMass + alphaE)
        b.lambdaEdge[e] += dl
        const k = dl * invMass / len
        x[a] += dx * k; x[a + 1] += dy * k; x[a + 2] += dz * k
        x[c] -= dx * k; x[c + 1] -= dy * k; x[c + 2] -= dz * k
      }
      for (let t = 0; t < restVol.length; t++) {
        const i0 = 3 * tets[4 * t], i1 = 3 * tets[4 * t + 1], i2 = 3 * tets[4 * t + 2], i3 = 3 * tets[4 * t + 3]
        const e1x = x[i1] - x[i0], e1y = x[i1 + 1] - x[i0 + 1], e1z = x[i1 + 2] - x[i0 + 2]
        const e2x = x[i2] - x[i0], e2y = x[i2 + 1] - x[i0 + 1], e2z = x[i2 + 2] - x[i0 + 2]
        const e3x = x[i3] - x[i0], e3y = x[i3 + 1] - x[i0 + 1], e3z = x[i3 + 2] - x[i0 + 2]
        // gradients: g1 = e2 x e3, g2 = e3 x e1, g3 = e1 x e2 (all / 6), g0 = -(g1 + g2 + g3)
        const g1x = (e2y * e3z - e2z * e3y) / 6, g1y = (e2z * e3x - e2x * e3z) / 6, g1z = (e2x * e3y - e2y * e3x) / 6
        const g2x = (e3y * e1z - e3z * e1y) / 6, g2y = (e3z * e1x - e3x * e1z) / 6, g2z = (e3x * e1y - e3y * e1x) / 6
        const g3x = (e1y * e2z - e1z * e2y) / 6, g3y = (e1z * e2x - e1x * e2z) / 6, g3z = (e1x * e2y - e1y * e2x) / 6
        const g0x = -(g1x + g2x + g3x), g0y = -(g1y + g2y + g3y), g0z = -(g1z + g2z + g3z)
        const vol = (g1x * e1x + g1y * e1y + g1z * e1z) // = e1 . (e2 x e3) / 6
        const C = vol - restVol[t]
        const wsum = invMass * (g0x * g0x + g0y * g0y + g0z * g0z + g1x * g1x + g1y * g1y + g1z * g1z + g2x * g2x + g2y * g2y + g2z * g2z + g3x * g3x + g3y * g3y + g3z * g3z)
        if (wsum < 1e-12) continue
        const dl = (-C - alphaV * b.lambdaVol[t]) / (wsum + alphaV)
        b.lambdaVol[t] += dl
        const k = dl * invMass
        x[i0] += g0x * k; x[i0 + 1] += g0y * k; x[i0 + 2] += g0z * k
        x[i1] += g1x * k; x[i1 + 1] += g1y * k; x[i1 + 2] += g1z * k
        x[i2] += g2x * k; x[i2 + 1] += g2y * k; x[i2 + 2] += g2z * k
        x[i3] += g3x * k; x[i3 + 1] += g3y * k; x[i3 + 2] += g3z * k
      }
    }
    if (DEBUG.on && DEBUG.track >= 0) { const q = 3 * DEBUG.track; DEBUG.log.push(`  after constraints x=${x[q].toFixed(6)},${x[q + 1].toFixed(6)},${x[q + 2].toFixed(6)} (prev ${xPrev[q].toFixed(6)},${xPrev[q + 1].toFixed(6)},${xPrev[q + 2].toFixed(6)})`) }
    b.shapeMatch(TUNING.shapeMatch)
    // collisions with MuJoCo geoms
    b.updateAabb()
    this.collide(b, d, dt)
    if (DEBUG.on && DEBUG.track >= 0) { const q = 3 * DEBUG.track; DEBUG.log.push(`  after collide     x=${x[q].toFixed(6)},${x[q + 1].toFixed(6)},${x[q + 2].toFixed(6)} v=${((x[q] - xPrev[q]) / dt).toFixed(4)},${((x[q + 1] - xPrev[q + 1]) / dt).toFixed(4)},${((x[q + 2] - xPrev[q + 2]) / dt).toFixed(4)}`) }
    // Velocities (damped). The normal push a contact gave a particle is not velocity (a plain
    // (x - xPrev) / dt turns a 20 mm push into 20 m/s and launches the bag); instead a touched particle may
    // not move into its surface: its velocity along the contact normal is at least the surface's own.
    const damp = Math.max(0, 1 - params.damping * dt)
    const { push, contactN, contactVn, inContact } = b
    for (let i = 0; i < n; i++) {
      const p = 3 * i
      v[p] = (x[p] - xPrev[p] - push[p]) / dt * damp
      v[p + 1] = (x[p + 1] - xPrev[p + 1] - push[p + 1]) / dt * damp
      v[p + 2] = (x[p + 2] - xPrev[p + 2] - push[p + 2]) / dt * damp
      if (inContact[i]) {
        const vn = v[p] * contactN[p] + v[p + 1] * contactN[p + 1] + v[p + 2] * contactN[p + 2] - contactVn[i]
        if (vn < 0) { v[p] -= vn * contactN[p]; v[p + 1] -= vn * contactN[p + 1]; v[p + 2] -= vn * contactN[p + 2] }
      }
    }
  }

  collide(b, d, dt) {
    const { m } = this
    const { x, xPrev, n, params, nSamples, sampleIdx, sampleW, sampleWsum } = b
    const aabb = b.aabb
    const r = PARTICLE_RADIUS
    const out = this.out, local = this.local, wn = this.worldN, vel = this.vel, pt = this.pt, prev = this.prev
    const mass = params.mass / n
    const gx = d.geom_xpos
    const total = n + nSamples
    for (const c of this.colliders) {
      const g3 = 3 * c.geom
      let cx, cy, cz, gm, g9
      if (c.virtual) { cx = c.virtual.pos[0]; cy = c.virtual.pos[1]; cz = c.virtual.pos[2]; gm = IDENTITY; g9 = 0 }
      else { cx = gx[g3]; cy = gx[g3 + 1]; cz = gx[g3 + 2]; gm = d.geom_xmat; g9 = 9 * c.geom }
      // broad phase: the collider's bounding sphere against the body's box (planes always pass)
      if (c.radius !== Infinity) {
        const R = c.radius + r
        if (cx + R < aabb[0] || cx - R > aabb[3] || cy + R < aabb[1] || cy - R > aabb[4] || cz + R < aabb[2] || cz - R > aabb[5]) continue
      }
      let touched = false
      let velRead = false
      let anyNear = false
      const near = b.near
      const cf = m.geom_friction[g3]
      const mu = c.priority > SOFT_PRIORITY ? cf : c.priority < SOFT_PRIORITY ? params.friction : Math.max(params.friction, cf)
      const reach2 = c.radius === Infinity ? Infinity : (c.radius + r + NEAR) * (c.radius + r + NEAR)
      for (let i = 0; i < total; i++) {
        if (i === n && !anyNear) break // no particle near this collider: no triangle can touch it either
        // the sample point and where it was at the start of the substep: a particle, or a point on a surface
        // triangle (barycentric weights over its three vertices)
        let sa, sb, sc, wa, wb, wc, wsum
        if (i < n) {
          const p = 3 * i
          pt[0] = x[p]; pt[1] = x[p + 1]; pt[2] = x[p + 2]
          // outside the collider's bounding sphere (plus the near margin): neither near nor touching
          const ddx = pt[0] - cx, ddy = pt[1] - cy, ddz = pt[2] - cz
          if (ddx * ddx + ddy * ddy + ddz * ddz > reach2) { near[i] = 0; continue }
          prev[0] = xPrev[p]; prev[1] = xPrev[p + 1]; prev[2] = xPrev[p + 2]
          sa = p; wa = 1; wsum = 1
        } else {
          const si = 3 * (i - n)
          const ia = sampleIdx[si], ib = sampleIdx[si + 1], ic = sampleIdx[si + 2]
          if (!near[ia] && !near[ib] && !near[ic]) continue
          sa = 3 * ia; sb = 3 * ib; sc = 3 * ic; wa = sampleW[si]; wb = sampleW[si + 1]; wc = sampleW[si + 2]
          wsum = sampleWsum[i - n]
          for (let k = 0; k < 3; k++) {
            pt[k] = wa * x[sa + k] + wb * x[sb + k] + wc * x[sc + k]
            prev[k] = wa * xPrev[sa + k] + wb * xPrev[sb + k] + wc * xPrev[sc + k]
          }
        }
        // world -> geom frame (xmat is row-major, columns are the geom axes): local = R^T (p - c)
        const dx = pt[0] - cx, dy = pt[1] - cy, dz = pt[2] - cz
        const lx = gm[g9] * dx + gm[g9 + 3] * dy + gm[g9 + 6] * dz
        const ly = gm[g9 + 1] * dx + gm[g9 + 4] * dy + gm[g9 + 7] * dz
        const lz = gm[g9 + 2] * dx + gm[g9 + 5] * dy + gm[g9 + 8] * dz
        local[0] = lx; local[1] = ly; local[2] = lz
        localDistance(c, local, out)
        if (i < n) { const isNear = out[0] < NEAR; near[i] = isNear ? 1 : 0; if (isNear) anyNear = true }
        const pen = r - out[0]
        if (!(pen > 0)) continue // also skips NaN (finite() ends the episode; no NaN force must reach MuJoCo)
        if (pen > this.maxPen) this.maxPen = pen
        // normal back to the world frame
        const nx = out[1], ny = out[2], nz = out[3]
        wn[0] = gm[g9] * nx + gm[g9 + 1] * ny + gm[g9 + 2] * nz
        wn[1] = gm[g9 + 3] * nx + gm[g9 + 4] * ny + gm[g9 + 5] * nz
        wn[2] = gm[g9 + 6] * nx + gm[g9 + 7] * ny + gm[g9 + 8] * nz
        const lift = Math.min(pen, MAX_LIFT)
        // friction against the surface's own motion at the contact point. d.cvel is the body's spatial
        // velocity [angular, linear] in the world frame, taken at its kinematic tree's subtree centre of mass
        if (!velRead) {
          vel.fill(0)
          if (c.dynamic) {
            const o = 6 * c.body
            for (let k = 0; k < 6; k++) vel[k] = d.cvel[o + k]
          } else if (c.virtual) {
            // rim velocity at the top of a roller: omega x (0, 0, R)
            const o = 6 * c.virtual.spinBody, R = c.virtual.spinRadius
            vel[3] = d.cvel[o + 1] * R; vel[4] = -d.cvel[o] * R; vel[5] = 0
          }
          velRead = true
        }
        let sx = vel[3], sy = vel[4], sz = vel[5]
        if (c.dynamic) {
          const root = 3 * m.body_rootid[c.body]
          const rx = pt[0] - d.subtree_com[root], ry = pt[1] - d.subtree_com[root + 1], rz = pt[2] - d.subtree_com[root + 2]
          sx += vel[1] * rz - vel[2] * ry; sy += vel[2] * rx - vel[0] * rz; sz += vel[0] * ry - vel[1] * rx
        }
        // correction of the sample point: out along the normal, then the tangential slip relative to the
        // surface removed up to the Coulomb limit
        let mx = wn[0] * lift, my = wn[1] * lift, mz = wn[2] * lift
        const sn = sx * wn[0] + sy * wn[1] + sz * wn[2] // surface speed along the normal
        {
          const { push, contactN, contactVn, inContact } = b
          const mark = (q, w) => { push[q] += mx * w; push[q + 1] += my * w; push[q + 2] += mz * w; const pi = q / 3; contactN[q] = wn[0]; contactN[q + 1] = wn[1]; contactN[q + 2] = wn[2]; contactVn[pi] = sn; inContact[pi] = 1 }
          if (i < n) mark(sa, 1)
          else { mark(sa, wa / wsum); mark(sb, wb / wsum); mark(sc, wc / wsum) }
        }
        let tx = (pt[0] + mx - prev[0]) - sx * dt, ty = (pt[1] + my - prev[1]) - sy * dt, tz = (pt[2] + mz - prev[2]) - sz * dt
        const tn = tx * wn[0] + ty * wn[1] + tz * wn[2]
        tx -= tn * wn[0]; ty -= tn * wn[1]; tz -= tn * wn[2]
        const slip = Math.sqrt(tx * tx + ty * ty + tz * tz)
        if (slip > 1e-12) {
          const limit = mu * lift
          const k = slip <= limit ? 1 : limit / slip
          mx -= tx * k; my -= ty * k; mz -= tz * k
          if (DEBUG.on) { DEBUG.slip += slip; DEBUG.fric += slip * k }
        }
        if (DEBUG.on) { DEBUG.contacts++; DEBUG.lift += lift; DEBUG.surf = [sx, sy, sz]; DEBUG.mu = mu; if (i === DEBUG.track) DEBUG.log.push(`    contact geom ${c.geom} type ${c.type} pen ${(pen * 1e6).toFixed(1)}um n=${wn.map(v => v.toFixed(3))} surf=${[sx, sy, sz].map(v => v.toFixed(4))} slip=${(slip * 1e6).toFixed(1)}um limit=${(mu * lift * 1e6).toFixed(1)}um`) }
        // apply to the vertices (a point on a triangle moves its vertices by their weights)
        if (i < n) { x[sa] += mx; x[sa + 1] += my; x[sa + 2] += mz }
        else {
          const ka = wa / wsum, kb = wb / wsum, kc = wc / wsum
          x[sa] += mx * ka; x[sa + 1] += my * ka; x[sa + 2] += mz * ka
          x[sb] += mx * kb; x[sb + 1] += my * kb; x[sb + 2] += mz * kb
          x[sc] += mx * kc; x[sc + 1] += my * kc; x[sc + 2] += mz * kc
        }
        touched = true
        // reaction on the body (see REACTION_SCALE): penalty along the normal on this substep's push, the
        // friction impulse as a force over the MuJoCo step; the sample carries one particle's mass or the
        // triangle's share
        if (c.dynamic) {
          const ms = i < n ? mass : mass / wsum
          // plus damping on the relative approach speed (surface toward the sample, along the normal that
          // points from the collider to the sample), never pulling
          const velN = i < n ? (b.v[sa] * wn[0] + b.v[sa + 1] * wn[1] + b.v[sa + 2] * wn[2]) : ((wa * b.v[sa] + wb * b.v[sb] + wc * b.v[sc]) * wn[0] + (wa * b.v[sa + 1] + wb * b.v[sb + 1] + wc * b.v[sc + 1]) * wn[1] + (wa * b.v[sa + 2] + wb * b.v[sb + 2] + wc * b.v[sc + 2]) * wn[2])
          const approach = Math.max(0, sn - velN)
          const fn = -(TUNING.reactionScale * ms * lift / (dt * dt) + TUNING.normalDamping * approach) / SUBSTEPS
          const scale = -ms / (dt * this.dtStep)
          const fx = (mx - wn[0] * lift) * scale + fn * wn[0], fy = (my - wn[1] * lift) * scale + fn * wn[1], fz = (mz - wn[2] * lift) * scale + fn * wn[2]
          const ix = d.xipos[3 * c.body], iy = d.xipos[3 * c.body + 1], iz = d.xipos[3 * c.body + 2]
          const ax = pt[0] - ix, ay = pt[1] - iy, az = pt[2] - iz
          const f = this.force, o = 6 * c.body
          f[o] += fx; f[o + 1] += fy; f[o + 2] += fz
          f[o + 3] += ay * fz - az * fy; f[o + 4] += az * fx - ax * fz; f[o + 5] += ax * fy - ay * fx
        }
      }
      if (touched) {
        const hand = this.handOfBody[c.body]
        if (hand >= 0) this.touching[hand] = 1
        if (c.dynamic) b.pressed = true
      }
    }
  }

  // Bag-bag contact: every sample of one bag is kept out of the other bag's 26-plane bound (both ways, half
  // the depth each), inelastically: the vertices moved lose their approach velocity toward the other bag.
  // Coarse (a bag is convex to the other), but two bags stack and push instead of merging or buzzing.
  softContacts(active) {
    const r = PARTICLE_RADIUS
    const pt = this.pt
    for (let a = 0; a < active.length; a++) for (let b = a + 1; b < active.length; b++) {
      const A = active[a], B = active[b]
      const ba = A.aabb, bb = B.aabb
      if (ba[3] + r < bb[0] || bb[3] + r < ba[0] || ba[4] + r < bb[1] || bb[4] + r < ba[1] || ba[5] + r < bb[2] || bb[5] + r < ba[2]) continue
      A.updateDop(); B.updateDop()
      const meanV = S => { let vx = 0, vy = 0, vz = 0; for (let i = 0; i < S.n; i++) { vx += S.v[3 * i]; vy += S.v[3 * i + 1]; vz += S.v[3 * i + 2] } return [vx / S.n, vy / S.n, vz / S.n] }
      const vA = meanV(A), vB = meanV(B)
      const mu = Math.max(A.params.friction, B.params.friction)
      const dt = this.dtStep / SUBSTEPS
      for (const [S, other, vOther] of [[A, B, vB], [B, A, vA]]) {
        const { x, xPrev, v, n, nSamples, sampleIdx, sampleW, sampleWsum } = S
        const planes = other.dop
        const total = n + nSamples
        for (let i = 0; i < total; i++) {
          let sa, sb, sc, wa, wb, wc, wsum
          if (i < n) { sa = 3 * i; wa = 1; wsum = 1; pt[0] = x[sa]; pt[1] = x[sa + 1]; pt[2] = x[sa + 2] }
          else {
            const si = 3 * (i - n)
            sa = 3 * sampleIdx[si]; sb = 3 * sampleIdx[si + 1]; sc = 3 * sampleIdx[si + 2]; wa = sampleW[si]; wb = sampleW[si + 1]; wc = sampleW[si + 2]
            wsum = sampleWsum[i - n]
            for (let k = 0; k < 3; k++) pt[k] = wa * x[sa + k] + wb * x[sb + k] + wc * x[sc + k]
          }
          let best = -Infinity, bi = 0
          for (let k = 0; k < NDOP; k++) {
            const d = DOP_DIRS[3 * k] * pt[0] + DOP_DIRS[3 * k + 1] * pt[1] + DOP_DIRS[3 * k + 2] * pt[2] - planes[k]
            if (d > best) { best = d; bi = k }
          }
          if (!(best < r)) continue
          const nx = DOP_DIRS[3 * bi], ny = DOP_DIRS[3 * bi + 1], nz = DOP_DIRS[3 * bi + 2]
          const h = (r - best) / 2
          const fix = (q, w) => {
            x[q] += nx * h * w; x[q + 1] += ny * h * w; x[q + 2] += nz * h * w
            // inelastic: no velocity toward the other bag along the push direction
            const rel = (v[q] - vOther[0]) * nx + (v[q + 1] - vOther[1]) * ny + (v[q + 2] - vOther[2]) * nz
            if (rel < 0) { v[q] -= rel * nx; v[q + 1] -= rel * ny; v[q + 2] -= rel * nz }
            // Coulomb friction: this substep's slip relative to the other bag, removed up to mu x push
            let tx = (x[q] - xPrev[q]) - vOther[0] * dt, ty = (x[q + 1] - xPrev[q + 1]) - vOther[1] * dt, tz = (x[q + 2] - xPrev[q + 2]) - vOther[2] * dt
            const tn = tx * nx + ty * ny + tz * nz
            tx -= tn * nx; ty -= tn * ny; tz -= tn * nz
            const slip = Math.sqrt(tx * tx + ty * ty + tz * tz)
            if (slip > 1e-12) {
              const k = Math.min(1, mu * h * w / slip)
              x[q] -= tx * k; x[q + 1] -= ty * k; x[q + 2] -= tz * k
              v[q] -= tx * k / dt; v[q + 1] -= ty * k / dt; v[q + 2] -= tz * k / dt
            }
          }
          if (i < n) fix(sa, 1)
          else { fix(sa, wa / wsum); fix(sb, wb / wsum); fix(sc, wc / wsum) }
        }
      }
      A.updateAabb(); B.updateAabb()
    }
  }

  /** Deepest particle penetration into any collider at the current state (m), without moving anything. */
  measurePenetration(d) {
    const saved = this.bodies.map(b => ({ x: Float64Array.from(b.x), xPrev: Float64Array.from(b.xPrev), v: Float64Array.from(b.v), aabb: Float64Array.from(b.aabb) }))
    const force = Float64Array.from(this.force), touching = Uint8Array.from(this.touching)
    this.maxPen = 0
    const dt = this.m.opt.timestep / SUBSTEPS
    for (const b of this.bodies) if (b.active) { b.updateAabb(); this.collide(b, d, dt) }
    const pen = this.maxPen
    this.bodies.forEach((b, i) => { b.x.set(saved[i].x); b.xPrev.set(saved[i].xPrev); b.v.set(saved[i].v); b.aabb.set(saved[i].aabb) })
    this.force.set(force)
    this.touching.set(touching)
    return pen
  }

  /** Renderer-side description: surface indices and UVs per body. */
  describe() {
    return this.bodies.map(b => ({
      name: b.name, n: b.n, surface: b.lattice.surface, topStart: b.lattice.topStart, topCount: b.lattice.topCount, uv: b.lattice.uv,
      half: b.def.half, cells: b.def.cells, label: b.def.label ?? null,
    }))
  }

  /** Header description for replay. */
  /** Header description for replay and offline rendering: lattice, per-episode params, surface triangles, top-face UVs and label (no JS needed to draw a bag). */
  header() {
    return this.bodies.map(b => ({ name: b.name, particles: b.n, half: b.def.half, cells: b.def.cells, ...b.params, surface: Array.from(b.lattice.surface), top_start: b.lattice.topStart, uv: Array.from(b.lattice.uv), label: b.def.label ?? null }))
  }

  /** Fraction of body a's particles inside body b's 26-plane bound (both must be up to date via updateDop). */
  overlap(a, b) {
    let inside = 0
    for (let i = 0; i < a.n; i++) {
      const px = a.x[3 * i], py = a.x[3 * i + 1], pz = a.x[3 * i + 2]
      let best = -Infinity
      for (let k = 0; k < NDOP; k++) { const d = DOP_DIRS[3 * k] * px + DOP_DIRS[3 * k + 1] * py + DOP_DIRS[3 * k + 2] * pz - b.dop[k]; if (d > best) best = d }
      if (best < 0) inside++
    }
    return inside / a.n
  }
}
