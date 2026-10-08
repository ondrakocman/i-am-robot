import tubeBox from './tube_box.js'
import conveyor from './conveyor.js'
import cubeSort from './cube_sort.js'

export const TASKS = { [tubeBox.name]: tubeBox, [conveyor.name]: conveyor, [cubeSort.name]: cubeSort }
export const DEFAULT_TASK = tubeBox.name

export function getTask(name) {
  const task = TASKS[name ?? DEFAULT_TASK]
  if (!task) throw new Error(`unknown task "${name}" (have: ${Object.keys(TASKS).join(', ')})`)
  return task
}
