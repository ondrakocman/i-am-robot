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

// Resolves when the transaction has committed; quota and commit failures surface as `abort`, not `error`
function run(mode, fn) {
  return db().then(d => new Promise((resolve, reject) => {
    const tx = d.transaction(STORE, mode)
    const req = fn(tx.objectStore(STORE))
    tx.oncomplete = () => resolve(req?.result)
    tx.onerror = () => reject(tx.error ?? new Error('IndexedDB transaction failed'))
    tx.onabort = () => reject(tx.error ?? new Error('IndexedDB transaction aborted'))
  }))
}

const listeners = new Set()
let stats = null
async function refreshStats() {
  const all = await run('readonly', s => s.getAll())
  stats = { total: all.length, success: all.filter(e => e.success).length }
  listeners.forEach(fn => fn(stats))
  return stats
}

/** Calls fn with { total, success } now and after every change. */
export function onEpisodesChanged(fn) {
  listeners.add(fn)
  if (stats) fn(stats)
  else refreshStats().catch(err => console.error('[episodes]', err))
  return () => listeners.delete(fn)
}

/** True if the browser granted persistent storage (otherwise it may evict the episodes under pressure). */
export async function requestPersistence() {
  if (!navigator.storage?.persist) return false
  return (await navigator.storage.persisted()) || navigator.storage.persist()
}

/** Resolves once the episode is committed to disk; rejects (e.g. quota exceeded) otherwise. */
export async function saveEpisode(header, buffer) {
  await run('readwrite', s => s.add({ header, success: header.success, task: header.task, data: new Blob([buffer]) }))
  if (stats) {
    stats = { total: stats.total + 1, success: stats.success + (header.success ? 1 : 0) }
    listeners.forEach(fn => fn(stats))
  }
}

export async function exportEpisodes() {
  const all = await run('readonly', s => s.getAll())
  return new Blob(all.map(e => e.data), { type: 'application/octet-stream' })
}

export async function clearEpisodes() {
  await run('readwrite', s => s.clear())
  await refreshStats()
}
