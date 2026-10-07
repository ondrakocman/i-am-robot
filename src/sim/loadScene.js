// Loads a task scene (+ the robot MJCF and its meshes) into MuJoCo's in-memory filesystem.
// `readFile(path)` resolves paths relative to public/ and returns a string or Uint8Array.

const ROBOT = 'mujoco/g1_upper.xml'
const MESH_DIR = 'models/meshes/'

export async function loadScene(mj, readFile, { scene = 'mujoco/tube_box.xml', timestep } = {}) {
  let xml = await readFile(scene)
  if (timestep) xml = xml.replace(/timestep="[^"]*"/, `timestep="${timestep}"`)
  const robot = await readFile(ROBOT)
  const meshes = [...new Set([...robot.matchAll(/<mesh[^>]*\bfile="([^"]+)"/g)].map(r => r[1]))]

  try { mj.FS.mkdirTree('/model/meshes') } catch { /* already exists */ }
  await Promise.all(meshes.map(async f => mj.FS.writeFile(`/model/meshes/${f}`, await readFile(MESH_DIR + f))))
  mj.FS.writeFile('/model/g1_upper.xml', robot)
  mj.FS.writeFile('/model/scene.xml', xml)
  return mj.MjModel.from_xml_path('/model/scene.xml')
}
