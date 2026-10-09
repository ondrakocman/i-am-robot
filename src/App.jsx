import { useEffect } from 'react'
import { Canvas } from '@react-three/fiber'
import { createXRStore, XR } from '@react-three/xr'
import * as THREE from 'three'
import { MujocoScene } from './components/MujocoScene.jsx'
import { onEpisodesChanged, episodeStats, exportEpisodes, clearEpisodes } from './sim/episodeStore.js'
import { TASKS, DEFAULT_TASK, hasTask } from './sim/tasks/index.js'

const REQUESTED_HZ = Number(new URLSearchParams(location.search).get('hz')) || 90
const xrStore = createXRStore({
  hand: { model: false },
  controller: false,
  // Desktop emulator (only injected on localhost): start in hand-tracking mode, the app ignores controllers
  emulate: { primaryInputMode: 'hand' },
  foveation: 1,
  // the display rate: 90 Hz by default (120 Hz halves the frame budget for the 630k-triangle robot); ?hz=72|90|120
  frameRate: rates => rates.includes(REQUESTED_HZ) ? REQUESTED_HZ : rates.includes(90) ? 90 : rates.at(-1),
})

let leaving = false // the operator confirmed a task switch; do not prompt again on the reload

export default function App() {
  useEffect(() => {
    const btn = document.getElementById('enter-vr')
    const status = document.getElementById('status')
    const instructions = document.getElementById('instructions')
    if (!btn) return

    if (!navigator.xr) {
      btn.textContent = 'WebXR N/A'
      btn.disabled = true
      if (status) status.textContent = 'Use the Meta Quest browser'
      return
    }
    if (instructions) instructions.textContent = `Open ${location.origin}${location.pathname} in the Meta Quest browser and allow hand tracking`

    navigator.xr.isSessionSupported('immersive-vr').then((supported) => {
      if (!supported) {
        btn.style.opacity = '0.5'
        if (status) status.textContent = 'Open on a Meta Quest browser'
      } else {
        if (status) status.textContent = 'Quest ready'
        if (instructions) instructions.style.display = 'none'
      }
    }).catch(err => { if (status) status.textContent = `WebXR unavailable: ${err.message}` })

    const enter = () => xrStore.enterVR().catch(err => { if (status) status.textContent = `Could not enter VR: ${err.message}` })
    btn.addEventListener('click', enter)
    return () => btn.removeEventListener('click', enter)
  }, [])

  // Task selector: the task is a URL parameter so the worker and a reload agree on it
  useEffect(() => {
    const select = document.getElementById('task')
    if (!select) return
    const params = new URLSearchParams(location.search)
    select.replaceChildren(...Object.values(TASKS).map(t => {
      const option = document.createElement('option')
      option.value = t.name
      option.textContent = t.title ?? t.instruction
      return option
    }))
    const requested = params.get('task')
    select.value = hasTask(requested) ? requested : DEFAULT_TASK
    if (requested && !hasTask(requested)) {
      // next to the selector, where the physics status line cannot overwrite it
      const note = document.createElement('span')
      note.textContent = ` unknown task "${requested}", showing ${select.value}`
      note.style.color = '#ffb35d'
      select.insertAdjacentElement('afterend', note)
    }
    const onChange = () => {
      // switching tasks reloads the page, which would drop an episode still being written or kept only in memory
      const { pending, unsaved } = episodeStats()
      if (pending + unsaved > 0 && !confirm('An episode is still being saved or has not been downloaded yet. Switching tasks reloads the page and loses it. Switch anyway?')) {
        select.value = hasTask(requested) ? requested : DEFAULT_TASK
        return
      }
      leaving = true
      params.set('task', select.value)
      location.search = params.toString()
    }
    select.addEventListener('change', onChange)
    return () => select.removeEventListener('change', onChange)
  }, [])

  useEffect(() => {
    const row = document.getElementById('episodes')
    if (!row) return
    row.hidden = false
    const count = document.getElementById('episode-count')
    const download = document.getElementById('download-episodes')
    const clear = document.getElementById('clear-episodes')
    const unsubscribe = onEpisodesChanged(({ total, success, unsaved, blocked }) => {
      count.textContent = (total ? `${total} episodes recorded (${success} successful)` : 'No episodes recorded yet')
        + (unsaved ? ` — ${unsaved} could not be stored, download now` : '')
        + (blocked ? ' — close the other I Am Robot tabs so episodes can be stored' : '')
      download.disabled = clear.disabled = total + unsaved === 0
    })
    // leaving the page while an episode is still being written, or exists only in memory, would lose it
    const onUnload = e => {
      const { pending, unsaved } = episodeStats()
      if (!leaving && pending + unsaved > 0) { e.preventDefault(); e.returnValue = '' }
    }
    window.addEventListener('beforeunload', onUnload)
    const onDownload = async () => {
      try {
        const { blob, partial, count } = await exportEpisodes()
        const url = URL.createObjectURL(blob)
        const a = document.createElement('a')
        a.href = url
        a.download = `iamr_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.iamr`
        a.click()
        setTimeout(() => URL.revokeObjectURL(url), 10000)
        if (partial) count.textContent = `Downloaded ${count} unsaved episode(s) only: the stored ones are locked by another I Am Robot tab. Close it and download again.`
      } catch (err) {
        count.textContent = `Export failed: ${err.message}`
      }
    }
    const onClear = () => { if (confirm('Delete all recorded episodes from this headset?')) clearEpisodes() }
    download.addEventListener('click', onDownload)
    clear.addEventListener('click', onClear)
    return () => {
      unsubscribe()
      window.removeEventListener('beforeunload', onUnload)
      download.removeEventListener('click', onDownload)
      clear.removeEventListener('click', onClear)
    }
  }, [])

  return (
    <Canvas
      style={{ position: 'fixed', inset: 0 }}
      gl={{
        antialias: true,
        alpha: false,
        powerPreference: 'high-performance',
        toneMapping: THREE.ACESFilmicToneMapping,
      }}
      shadows="percentage"
      camera={{
        fov: 75,
        near: 0.01,
        far: 100,
        position: [0, 1.24, 2],
      }}
    >
      <color attach="background" args={['#607080']} />
      <XR store={xrStore}>
        <MujocoScene />
      </XR>
    </Canvas>
  )
}
