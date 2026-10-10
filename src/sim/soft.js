// Soft parcels: XPBD (extended position-based dynamics) lattices stepped in lockstep with MuJoCo. MuJoCo's own
// deformables (flex) cost too much on the headset; this is a few hundred constraints per parcel, pure JS, and
// deterministic (float64, fixed iteration order, sqrt only), so a recorded episode replays it bit for bit like
// the rigid state.
//
// Coupling with MuJoCo is through its compiled geoms: every colliding geom (robot hull meshes as fitted boxes,
// scene boxes, cylinders, planes, spheres, capsules, the rigid parcels) is a kinematic collider for the
// particles, with Coulomb friction against the surface's actual velocity (a turning roller carries a bag), and
// the contact impulses are applied back to MuJoCo bodies through xfrc_applied, so a hand carries the bag's
// weight and a bag can push a rigid parcel. Soft-soft contact is particle-sphere repulsion when two bags'
// bounding boxes overlap.

const GRAVITY = -9.81
const PARTICLE_RADIUS = 0.004   // contact offset: a bag's skin
const SUBSTEPS = 2              // per MuJoCo step (1 ms at the default 2 ms step)
const ITERATIONS = 2            // constraint passes per substep
const MAX_LIFT = 0.02           // a particle is pushed out of a collider by at most this per substep (tunnelling guard)
const SOFT_PRIORITY = 2         // like the rigid objects: a higher-priority geom's friction wins (the chute), a tie takes the larger
// Reaction on MuJoCo bodies: the normal part is a penalty on how far a contact pushed the sample this substep,
// with stiffness REACTION_SCALE x (sample mass / substep^2): that is REACTION_SCALE times the impulse the
// sample actually received, so the transferred force is consistent across bag mass and timestep (at rest it
// is REACTION_SCALE x the resting weight on the body). The full impulse is an explicit, stiff exchange that
// bounces a 400 g box and launches it (measured, with and without low-pass filtering); 0.15 is stable for the
// bodies in these scenes. The tangential (friction) part is the sample's impulse as a force over the step.
export const REACTION_SCALE = 0.15
export const DEBUG = { on: false, contacts: 0, lift: 0, slip: 0, fric: 0, surf: [0, 0, 0], mu: 0, track: -1, log: [] }
// Collision samples: the particles plus, on every surface triangle, its centroid and three edge midpoints.
// Particles alone (radius 4 mm, spacing 25 mm) let a 17 mm finger pass between them and impale the bag.
const SURFACE_SAMPLES = [[1 / 3, 1 / 3, 1 / 3], [0.5, 0.5, 0], [0, 0.5, 0.5], [0.5, 0, 0.5]]
const NEAR = 0.03               // a triangle's samples are only tested against a collider one of its vertices is this close to
const PLANE = 0, SPHERE = 2, CAPSULE = 3, CYLINDER = 5, BOX = 6, MESH = 7
const IDENTITY = Float64Array.from([1, 0, 0, 0, 1, 0, 0, 0, 1])

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
    // (vertex a, b, c, weights) per surface sample, for collision
    const tris = this.lattice.surface
    this.samples = []
    for (let t = 0; t < tris.length; t += 3) for (const w of SURFACE_SAMPLES) this.samples.push([tris[t], tris[t + 1], tris[t + 2], w[0], w[1], w[2]])
    this.near = new Uint8Array(this.n) // per collider pass: particles within NEAR of it
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
    const [a, b, , d] = this.lattice.topCorners
    const { x } = this
    const u = [x[3 * b] - x[3 * a], x[3 * b + 1] - x[3 * a + 1], x[3 * b + 2] - x[3 * a + 2]]
    const v = [x[3 * d] - x[3 * a], x[3 * d + 1] - x[3 * a + 1], x[3 * d + 2] - x[3 * a + 2]]
    const nz = u[0] * v[1] - u[1] * v[0]
    const nx = u[1] * v[2] - u[2] * v[1], ny = u[2] * v[0] - u[0] * v[2]
    const len = Math.sqrt(nx * nx + ny * ny + nz * nz)
    return len > 0 ? nz / len : 0
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
    default: { // BOX, and MESH as its fitted box (c.half, centred at c.center in the geom frame)
      const half = c.type === BOX ? c.size : c.half
      const cx = c.type === BOX ? 0 : c.center[0], cy = c.type === BOX ? 0 : c.center[1], cz = c.type === BOX ? 0 : c.center[2]
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
  constructor(mj, m, defs, { handOfBody, belts = [] } = {}) {
    this.mj = mj
    this.m = m
    this.belts = belts
    this.bodies = Object.entries(defs).map(([name, def]) => new SoftBody(name, def))
    this.byName = Object.fromEntries(this.bodies.map(b => [b.name, b]))
    this.total = this.bodies.reduce((s, b) => s + b.n, 0)
    this.handOfBody = handOfBody ?? new Int8Array(m.nbody).fill(-1)
    this.touching = new Uint8Array(2)
    this.colliders = this.buildColliders()
    this.force = new Float64Array(6 * m.nbody)   // reaction this step, written to xfrc_applied
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
    const cov = new Float64Array(9), axes = new Float64Array(9)
    const rollers = this.belts.map(() => [])
    for (let g = 0; g < m.ngeom; g++) {
      if (!m.geom_contype[g] && !m.geom_conaffinity[g]) continue
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
        const mean = [0, 0, 0]
        for (let i = 0; i < vn; i++) { mean[0] += vert[3 * i]; mean[1] += vert[3 * i + 1]; mean[2] += vert[3 * i + 2] }
        mean[0] /= vn; mean[1] /= vn; mean[2] /= vn
        cov.fill(0)
        for (let i = 0; i < vn; i++) {
          const d = [vert[3 * i] - mean[0], vert[3 * i + 1] - mean[1], vert[3 * i + 2] - mean[2]]
          for (let r = 0; r < 3; r++) for (let s = 0; s < 3; s++) cov[3 * r + s] += d[r] * d[s]
        }
        eigen3(cov, axes)
        // box in the eigenframe: centre = mid of projections, half = half range
        const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity]
        for (let i = 0; i < vn; i++) for (let a = 0; a < 3; a++) {
          const p = vert[3 * i] * axes[a] + vert[3 * i + 1] * axes[3 + a] + vert[3 * i + 2] * axes[6 + a]
          if (p < lo[a]) lo[a] = p
          if (p > hi[a]) hi[a] = p
        }
        c.axes = Float64Array.from(axes) // columns = box axes in the geom frame
        c.half = [0, 1, 2].map(a => (hi[a] - lo[a]) / 2)
        c.center = [0, 1, 2].map(a => (hi[a] + lo[a]) / 2) // in the eigenframe
        c.radius = Math.sqrt(c.center[0] ** 2 + c.center[1] ** 2 + c.center[2] ** 2) + Math.sqrt(c.half[0] ** 2 + c.half[1] ** 2 + c.half[2] ** 2)
      } else if (type === PLANE) c.radius = Infinity
      else if (type === BOX) c.radius = Math.sqrt(c.size[0] ** 2 + c.size[1] ** 2 + c.size[2] ** 2)
      else if (type === CYLINDER || type === CAPSULE) c.radius = Math.sqrt(c.size[0] ** 2 + c.size[1] ** 2) + (type === CAPSULE ? c.size[0] : 0)
      else c.radius = c.size[0]
      list.push(c)
    }
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
    if (active.length) {
      for (let s = 0; s < SUBSTEPS; s++) {
        for (const b of active) this.substep(b, d, dt)
        if (active.length > 1) this.softContacts(active, dt)
      }
    }
    d.xfrc_applied.set(this.force)
  }

  substep(b, d, dt) {
    const { x, xPrev, v, n, invMass, lattice, params } = b
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
    // collisions with MuJoCo geoms
    b.updateAabb()
    this.collide(b, d, dt)
    if (DEBUG.on && DEBUG.track >= 0) { const q = 3 * DEBUG.track; DEBUG.log.push(`  after collide     x=${x[q].toFixed(6)},${x[q + 1].toFixed(6)},${x[q + 2].toFixed(6)} v=${((x[q] - xPrev[q]) / dt).toFixed(4)},${((x[q + 1] - xPrev[q + 1]) / dt).toFixed(4)},${((x[q + 2] - xPrev[q + 2]) / dt).toFixed(4)}`) }
    // velocities, damping
    const damp = Math.max(0, 1 - params.damping * dt)
    for (let i = 0; i < 3 * n; i++) v[i] = (x[i] - xPrev[i]) / dt * damp
  }

  collide(b, d, dt) {
    const { m } = this
    const { x, xPrev, n, params, samples } = b
    const aabb = b.aabb
    const r = PARTICLE_RADIUS
    const out = this.out, local = this.local, wn = this.worldN, vel = this.vel, pt = this.pt, prev = this.prev
    const mass = params.mass / n
    const gx = d.geom_xpos
    const total = n + samples.length
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
      for (let i = 0; i < total; i++) {
        if (i === n && !anyNear) break // no particle near this collider: no triangle can touch it either
        // the sample point and where it was at the start of the substep: a particle, or a point on a surface
        // triangle (barycentric weights over its three vertices)
        let sa, sb, sc, wa, wb, wc, wsum
        if (i < n) {
          const p = 3 * i
          pt[0] = x[p]; pt[1] = x[p + 1]; pt[2] = x[p + 2]
          prev[0] = xPrev[p]; prev[1] = xPrev[p + 1]; prev[2] = xPrev[p + 2]
          sa = p; wa = 1; wsum = 1
        } else {
          const smp = samples[i - n]
          if (!near[smp[0]] && !near[smp[1]] && !near[smp[2]]) continue
          sa = 3 * smp[0]; sb = 3 * smp[1]; sc = 3 * smp[2]; wa = smp[3]; wb = smp[4]; wc = smp[5]
          wsum = wa * wa + wb * wb + wc * wc
          for (let k = 0; k < 3; k++) {
            pt[k] = wa * x[sa + k] + wb * x[sb + k] + wc * x[sc + k]
            prev[k] = wa * xPrev[sa + k] + wb * xPrev[sb + k] + wc * xPrev[sc + k]
          }
        }
        // world -> geom frame (xmat is row-major, columns are the geom axes): local = R^T (p - c)
        const dx = pt[0] - cx, dy = pt[1] - cy, dz = pt[2] - cz
        let lx = gm[g9] * dx + gm[g9 + 3] * dy + gm[g9 + 6] * dz
        let ly = gm[g9 + 1] * dx + gm[g9 + 4] * dy + gm[g9 + 7] * dz
        let lz = gm[g9 + 2] * dx + gm[g9 + 5] * dy + gm[g9 + 8] * dz
        if (c.type === MESH) { // and into the fitted box's eigenframe
          const a = c.axes
          const ex = a[0] * lx + a[3] * ly + a[6] * lz, ey = a[1] * lx + a[4] * ly + a[7] * lz, ez = a[2] * lx + a[5] * ly + a[8] * lz
          lx = ex; ly = ey; lz = ez
        }
        local[0] = lx; local[1] = ly; local[2] = lz
        localDistance(c, local, out)
        if (i < n) { const isNear = out[0] < NEAR; near[i] = isNear ? 1 : 0; if (isNear) anyNear = true }
        const pen = r - out[0]
        if (pen <= 0) continue
        if (pen > this.maxPen) this.maxPen = pen
        // normal back to the world frame
        let nx = out[1], ny = out[2], nz = out[3]
        if (c.type === MESH) {
          const a = c.axes
          const ex = a[0] * nx + a[1] * ny + a[2] * nz, ey = a[3] * nx + a[4] * ny + a[5] * nz, ez = a[6] * nx + a[7] * ny + a[8] * nz
          nx = ex; ny = ey; nz = ez
        }
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
          const fn = -REACTION_SCALE * ms * lift / (dt * dt) / SUBSTEPS
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
      }
    }
  }

  // Bag-bag contact: every sample of one bag is kept out of the other bag's bounding box (both ways, half the
  // depth each). Coarse (a bag is treated as its box by the other), but two bags stack and push instead of
  // merging.
  softContacts(active, dt) {
    const r = PARTICLE_RADIUS
    const pt = this.pt
    for (let a = 0; a < active.length; a++) for (let b = a + 1; b < active.length; b++) {
      const A = active[a], B = active[b]
      const ba = A.aabb, bb = B.aabb
      if (ba[3] + r < bb[0] || bb[3] + r < ba[0] || ba[4] + r < bb[1] || bb[4] + r < ba[1] || ba[5] + r < bb[2] || bb[5] + r < ba[2]) continue
      for (const [S, box] of [[A, bb], [B, ba]]) {
        const { x, n, samples } = S
        const total = n + samples.length
        const hx = (box[3] - box[0]) / 2, hy = (box[4] - box[1]) / 2, hz = (box[5] - box[2]) / 2
        const cx = (box[3] + box[0]) / 2, cy = (box[4] + box[1]) / 2, cz = (box[5] + box[2]) / 2
        for (let i = 0; i < total; i++) {
          let sa, sb, sc, wa, wb, wc, wsum
          if (i < n) { sa = 3 * i; wa = 1; wsum = 1; pt[0] = x[sa]; pt[1] = x[sa + 1]; pt[2] = x[sa + 2] }
          else {
            const smp = samples[i - n]
            sa = 3 * smp[0]; sb = 3 * smp[1]; sc = 3 * smp[2]; wa = smp[3]; wb = smp[4]; wc = smp[5]
            wsum = wa * wa + wb * wb + wc * wc
            for (let k = 0; k < 3; k++) pt[k] = wa * x[sa + k] + wb * x[sb + k] + wc * x[sc + k]
          }
          const qx = Math.abs(pt[0] - cx) - hx, qy = Math.abs(pt[1] - cy) - hy, qz = Math.abs(pt[2] - cz) - hz
          if (qx > r || qy > r || qz > r) continue
          // inside (or within r of) the box: out through the nearest face, half the depth
          let mx = 0, my = 0, mz = 0
          if (qx >= qy && qx >= qz) mx = (pt[0] >= cx ? 1 : -1) * (r - qx) / 2
          else if (qy >= qz) my = (pt[1] >= cy ? 1 : -1) * (r - qy) / 2
          else mz = (pt[2] >= cz ? 1 : -1) * (r - qz) / 2
          if (i < n) { x[sa] += mx; x[sa + 1] += my; x[sa + 2] += mz }
          else {
            const ka = wa / wsum, kb = wb / wsum, kc = wc / wsum
            x[sa] += mx * ka; x[sa + 1] += my * ka; x[sa + 2] += mz * ka
            x[sb] += mx * kb; x[sb + 1] += my * kb; x[sb + 2] += mz * kb
            x[sc] += mx * kc; x[sc + 1] += my * kc; x[sc + 2] += mz * kc
          }
        }
      }
      for (const S of [A, B]) {
        const damp = Math.max(0, 1 - S.params.damping * dt)
        for (let i = 0; i < 3 * S.n; i++) S.v[i] = (S.x[i] - S.xPrev[i]) / dt * damp
        S.updateAabb()
      }
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
  header() {
    return this.bodies.map(b => ({ name: b.name, particles: b.n, half: b.def.half, cells: b.def.cells, ...b.params }))
  }
}
