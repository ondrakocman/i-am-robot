import tubeBox from './tube_box.js'
import packageSort from './package_sort.js'

export const TASKS = { [tubeBox.name]: tubeBox, [packageSort.name]: packageSort }
export const DEFAULT_TASK = tubeBox.name

export function getTask(name) {
  const task = TASKS[name ?? DEFAULT_TASK]
  if (!task) throw new Error(`unknown task "${name}" (have: ${Object.keys(TASKS).join(', ')})`)
  return task
}
