// Loads a task scene (+ the robot MJCF and its meshes) into MuJoCo's in-memory filesystem and returns the
// compiled model together with a SHA-256 manifest of every file that went into it, which episodes record so a
// dataset can always be tied to the exact assets that produced it.
// `readFile(path)` resolves paths relative to public/ and returns a string (.xml) or Uint8Array.

const ROBOT = 'mujoco/g1_upper.xml'
const MESH_DIR = 'models/meshes/'
const VFS_ROOT = '/model'

const MESH_FILE = /<mesh[^>]*\bfile="([^"]+)"/g

async function sha256(data) {
  const bytes = typeof data === 'string' ? new TextEncoder().encode(data) : data
  const digest = await crypto.subtle.digest('SHA-256', bytes)
  return Array.from(new Uint8Array(digest), b => b.toString(16).padStart(2, '0')).join('')
}

function mkdirTree(mj, path) {
  try { mj.FS.mkdirTree(path) } catch { /* already exists */ }
}

export async function loadScene(mj, readFile, { scene, timestep } = {}) {
  if (!scene) throw new Error('loadScene: scene path required')
  const assets = {}
  const read = async path => {
    const data = await readFile(path)
    assets[path] = await sha256(data)
    return data
  }

  const xml = await read(scene)
  const robot = await read(ROBOT)
  const meshFiles = src => [...new Set([...src.matchAll(MESH_FILE)].map(r => r[1]))]
  // The robot's compiler sets meshdir="meshes". Scene meshes live under public/models/objects/<name>/ and are
  // referenced as "objects/<name>/<file>", so both are mirrored into the same virtual meshdir.
  const files = [
    ...meshFiles(robot).map(f => ({ src: MESH_DIR + f, dst: f })),
    ...meshFiles(xml).map(f => ({ src: 'models/' + f, dst: f })),
  ]
  for (const { dst } of files) mkdirTree(mj, `${VFS_ROOT}/meshes/${dst}`.split('/').slice(0, -1).join('/'))
  await Promise.all(files.map(async ({ src, dst }) => mj.FS.writeFile(`${VFS_ROOT}/meshes/${dst}`, await read(src))))
  mj.FS.writeFile(`${VFS_ROOT}/g1_upper.xml`, robot)
  mj.FS.writeFile(`${VFS_ROOT}/scene.xml`, xml)
  const model = mj.MjModel.from_xml_path(`${VFS_ROOT}/scene.xml`)
  // the compiled model holds its own copy of everything: free the in-memory file copies (tens of MB of meshes)
  for (const { dst } of files) mj.FS.unlink(`${VFS_ROOT}/meshes/${dst}`)
  mj.FS.unlink(`${VFS_ROOT}/g1_upper.xml`)
  mj.FS.unlink(`${VFS_ROOT}/scene.xml`)
  if (timestep) model.opt.timestep = timestep // m.opt is a reference view: assignment writes through
  return { model, assets }
}
