// Loads a task scene (+ the robot MJCF and its meshes) into MuJoCo's in-memory filesystem.
// `readFile(path)` resolves paths relative to public/ and returns a string or Uint8Array.

const ROBOT = 'mujoco/g1_upper.xml'
const MESH_DIR = 'models/meshes/'

export async function loadScene(mj, readFile, { scene = 'mujoco/tube_box.xml', timestep } = {}) {
  let xml = await readFile(scene)
  if (timestep) xml = xml.replace(/timestep="[^"]*"/, `timestep="${timestep}"`)
  const robot = await readFile(ROBOT)
  const meshFiles = src => [...new Set([...src.matchAll(/<mesh[^>]*\bfile="([^"]+)"/g)].map(r => r[1]))]
  // The robot's compiler sets meshdir="meshes"; scene meshes live under public/models/objects/... and are
  // referenced as "objects/<name>/<file>", so they are mirrored into the same virtual meshdir
  const robotMeshes = meshFiles(robot).map(f => ({ src: MESH_DIR + f, dst: f }))
  const sceneMeshes = meshFiles(xml).map(f => ({ src: 'models/' + f, dst: f }))

  for (const { dst } of sceneMeshes) {
    try { mj.FS.mkdirTree('/model/meshes/' + dst.split('/').slice(0, -1).join('/')) } catch { /* exists */ }
  }
  try { mj.FS.mkdirTree('/model/meshes') } catch { /* already exists */ }
  await Promise.all([...robotMeshes, ...sceneMeshes].map(async ({ src, dst }) => mj.FS.writeFile(`/model/meshes/${dst}`, await readFile(src))))
  mj.FS.writeFile('/model/g1_upper.xml', robot)
  mj.FS.writeFile('/model/scene.xml', xml)
  return mj.MjModel.from_xml_path('/model/scene.xml')
}
