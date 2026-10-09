// Recorded episodes live in IndexedDB on the headset until downloaded. The download is one .iamr file:
// the episode chunks concatenated (see episode.js and scripts/load_episodes.py).

const DB_NAME = 'i-am-robot'
const DB_VERSION = 2
const STORE = 'episodes'

let dbPromise = null
function db() {
  dbPromise ??= new Promise((resolve, reject) => {
    const req = indexedDB.open(DB_NAME, DB_VERSION)
    req.onupgradeneeded = () => {
      const d = req.result
      const store = d.objectStoreNames.contains(STORE) ? req.transaction.objectStore(STORE) : d.createObjectStore(STORE, { keyPath: 'id', autoIncrement: true })
      if (!store.indexNames.contains('success')) store.createIndex('success', 'success')
    }
    req.onsuccess = () => {
      const d = req.result
      // another tab upgrading the schema: let go of our connection so it can proceed
      d.onversionchange = () => { d.close(); dbPromise = null }
      resolve(d)
    }
    req.onerror = () => { dbPromise = null; reject(req.error) }
    req.onblocked = () => reject(new Error('IndexedDB upgrade blocked by another tab'))
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
  const [total, success] = await Promise.all([run('readonly', s => s.count()), run('readonly', s => s.index('success').count(IDBKeyRange.only(1)))])
  stats = { total, success }
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
  // the success index needs a key, so booleans are stored as 0/1
  await run('readwrite', s => s.add({ header, success: header.success ? 1 : 0, task: header.task, data: new Blob([buffer]) }))
  if (stats) {
    stats = { total: stats.total + 1, success: stats.success + (header.success ? 1 : 0) }
    listeners.forEach(fn => fn(stats))
  }
}

/** One Blob of every stored episode, in recording order. */
export async function exportEpisodes() {
  const all = await run('readonly', s => s.getAll())
  return new Blob(all.map(e => e.data), { type: 'application/octet-stream' })
}

export async function clearEpisodes() {
  await run('readwrite', s => s.clear())
  await refreshStats()
}
