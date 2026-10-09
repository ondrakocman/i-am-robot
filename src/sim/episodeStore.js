// Recorded episodes live in IndexedDB on the headset until downloaded. The download is one .iamr file:
// the episode chunks concatenated (see episode.js and scripts/load_episodes.py). Every episode stays in
// memory until its write has committed; episodes the database refuses (quota, eviction, storage disabled, an
// upgrade blocked by an older tab) stay there and are still part of the next download.

const DB_NAME = 'i-am-robot'
const DB_VERSION = 2
const STORE = 'episodes'
/** An older tab holding the database open this long counts as a failure (saves fall back to memory). */
export const BLOCKED_TIMEOUT_MS = 2000

let dbPromise = null
function db() {
  dbPromise ??= new Promise((resolve, reject) => {
    const req = indexedDB.open(DB_NAME, DB_VERSION)
    let blockedTimer = null
    let gaveUp = false
    req.onupgradeneeded = e => {
      const d = req.result
      const store = d.objectStoreNames.contains(STORE) ? req.transaction.objectStore(STORE) : d.createObjectStore(STORE, { keyPath: 'id', autoIncrement: true })
      if (!store.indexNames.contains('success')) store.createIndex('success', 'success')
      if (e.oldVersion > 0 && e.oldVersion < 2) {
        // v1 stored `success` as a boolean, which IndexedDB cannot index: rewrite the old records as 0/1
        store.openCursor().onsuccess = ev => {
          const c = ev.target.result
          if (!c) return
          if (typeof c.value.success === 'boolean') c.update({ ...c.value, success: c.value.success ? 1 : 0 })
          c.continue()
        }
      }
    }
    req.onsuccess = () => {
      clearTimeout(blockedTimer)
      const d = req.result
      if (gaveUp) { d.close(); return } // the blocked open finished after we stopped waiting; the next call reopens
      // another tab upgrading the schema: let go of our connection so it can proceed
      d.onversionchange = () => { d.close(); dbPromise = null }
      stats = { ...stats, blocked: false }
      resolve(d)
    }
    req.onerror = () => { clearTimeout(blockedTimer); dbPromise = null; reject(req.error) }
    // An older tab still has the previous schema open; the open completes once it lets go. If that does not
    // happen soon, give up so saves fall back to memory instead of waiting forever; a later call retries.
    req.onblocked = () => {
      stats = { ...stats, blocked: true }
      notify()
      blockedTimer = setTimeout(() => {
        gaveUp = true
        dbPromise = null
        reject(new Error('another I Am Robot tab is holding the episode database open; close it'))
      }, BLOCKED_TIMEOUT_MS)
    }
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
const pending = new Set() // writes in flight, each { data, done }
const unsaved = []        // Blobs the database refused, in recording order
// available: null until the first open settles; pending/unsaved: episodes that exist only in this page's memory
let stats = { total: 0, success: 0, pending: 0, unsaved: 0, blocked: false, available: null }
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

/** The current stats (for one-off checks; onEpisodesChanged for updates). */
export const episodeStats = () => stats

/** True if the browser granted persistent storage (otherwise it may evict the episodes under pressure). */
export async function requestPersistence() {
  if (!navigator.storage?.persist) return false
  return (await navigator.storage.persisted()) || navigator.storage.persist()
}

/**
 * Resolves once the episode is committed to disk. Until then it is held in memory; if the database refuses
 * it, it stays there for the next download and the promise rejects so the UI can say so.
 */
export function saveEpisode(header, data) {
  const entry = { data }
  entry.done = (async () => {
    try {
      await run('readwrite', s => s.add({ header, success: header.success ? 1 : 0, task: header.task, data }))
      stats = { ...stats, total: stats.total + 1, success: stats.success + (header.success ? 1 : 0), available: true }
    } catch (err) {
      unsaved.push(data)
      stats = { ...stats, unsaved: unsaved.length }
      throw err
    } finally {
      pending.delete(entry)
      stats = { ...stats, pending: pending.size }
      notify()
    }
  })()
  pending.add(entry)
  stats = { ...stats, pending: pending.size }
  notify()
  return entry.done
}

/**
 * One Blob of every episode in recording order: the stored ones, then any the database refused. Writes still
 * in flight are waited for first, so each episode is in exactly one of the two groups.
 */
export async function exportEpisodes() {
  await Promise.allSettled([...pending].map(p => p.done))
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
