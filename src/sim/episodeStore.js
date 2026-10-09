// Recorded episodes live in IndexedDB on the headset until downloaded. The download is one .iamr file:
// the episode chunks concatenated (see episode.js and scripts/load_episodes.py). Episodes the database
// refuses (quota, eviction, storage disabled, an upgrade blocked by another tab) stay in memory and are
// still part of the next download.

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
    // `blocked` is not a failure: the open completes once the other tab lets go. Just say so meanwhile.
    req.onblocked = () => { stats = { ...stats, blocked: true }; notify() }
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
const unsaved = [] // Blobs the database refused, in recording order
let stats = { total: 0, success: 0, unsaved: 0, blocked: false, available: null } // available: null until the first open settles
const notify = () => listeners.forEach(fn => fn(stats))

async function refreshStats() {
  try {
    const [total, success] = await Promise.all([run('readonly', s => s.count()), run('readonly', s => s.index('success').count(IDBKeyRange.only(1)))])
    stats = { ...stats, total, success, blocked: false, available: true }
  } catch (err) {
    console.error('[episodes] storage unavailable', err)
    stats = { ...stats, available: false }
  }
  notify()
  return stats
}

/** Calls fn with the current stats now and after every change. */
export function onEpisodesChanged(fn) {
  listeners.add(fn)
  fn(stats)
  if (stats.available === null) refreshStats()
  return () => listeners.delete(fn)
}

/** True if the browser granted persistent storage (otherwise it may evict the episodes under pressure). */
export async function requestPersistence() {
  if (!navigator.storage?.persist) return false
  return (await navigator.storage.persisted()) || navigator.storage.persist()
}

/**
 * Resolves once the episode is committed to disk. If the database refuses it, the episode is kept in memory
 * for the next download and the promise rejects so the UI can say so.
 */
export async function saveEpisode(header, data) {
  try {
    // the success index needs a key, so booleans are stored as 0/1
    await run('readwrite', s => s.add({ header, success: header.success ? 1 : 0, task: header.task, data }))
    stats = { ...stats, total: stats.total + 1, success: stats.success + (header.success ? 1 : 0), available: true }
  } catch (err) {
    unsaved.push(data)
    stats = { ...stats, unsaved: unsaved.length }
    throw err
  } finally {
    notify()
  }
}

/** One Blob of every stored episode in recording order, followed by any the database refused. */
export async function exportEpisodes() {
  let stored = []
  try {
    stored = await run('readonly', s => s.getAll())
  } catch (err) {
    console.error('[episodes] storage unavailable, exporting the in-memory episodes only', err)
  }
  return new Blob([...stored.map(e => e.data), ...unsaved], { type: 'application/octet-stream' })
}

export async function clearEpisodes() {
  unsaved.length = 0
  stats = { ...stats, unsaved: 0 }
  try { await run('readwrite', s => s.clear()) } catch (err) { console.error('[episodes] clear failed', err) }
  await refreshStats()
}
