// Recorded episodes live in IndexedDB on the headset until downloaded. The download is one .iamr file:
// the episode chunks concatenated (see episode.js and scripts/load_episodes.py).

const DB_NAME = 'i-am-robot'
const STORE = 'episodes'

let dbPromise = null
function db() {
  dbPromise ??= new Promise((resolve, reject) => {
    const req = indexedDB.open(DB_NAME, 1)
    req.onupgradeneeded = () => req.result.createObjectStore(STORE, { keyPath: 'id', autoIncrement: true })
    req.onsuccess = () => resolve(req.result)
    req.onerror = () => reject(req.error)
  })
  return dbPromise
}

function run(mode, fn) {
  return db().then(d => new Promise((resolve, reject) => {
    const tx = d.transaction(STORE, mode)
    const req = fn(tx.objectStore(STORE))
    tx.oncomplete = () => resolve(req?.result)
    tx.onerror = () => reject(tx.error)
  }))
}

const listeners = new Set()
const notify = () => countEpisodes().then(stats => listeners.forEach(fn => fn(stats)))

export function onEpisodesChanged(fn) {
  listeners.add(fn)
  countEpisodes().then(fn)
  return () => listeners.delete(fn)
}

export async function saveEpisode(header, buffer) {
  navigator.storage?.persist?.()
  await run('readwrite', s => s.add({ header, success: header.success, data: new Blob([buffer]) }))
  notify()
}

export async function countEpisodes() {
  const all = await run('readonly', s => s.getAll())
  return { total: all.length, success: all.filter(e => e.success).length }
}

export async function exportEpisodes({ successOnly = false } = {}) {
  const all = await run('readonly', s => s.getAll())
  return new Blob(all.filter(e => !successOnly || e.success).map(e => e.data), { type: 'application/octet-stream' })
}

export async function clearEpisodes() {
  await run('readwrite', s => s.clear())
  notify()
}
