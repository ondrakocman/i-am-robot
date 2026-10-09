import { useEffect } from 'react'
import { Canvas } from '@react-three/fiber'
import { createXRStore, XR } from '@react-three/xr'
import * as THREE from 'three'
import { MujocoScene } from './components/MujocoScene.jsx'
import { onEpisodesChanged, exportEpisodes, clearEpisodes } from './sim/episodeStore.js'
import { TASKS, DEFAULT_TASK } from './sim/tasks/index.js'

const xrStore = createXRStore({
  hand: { model: false },
  controller: false,
  // Desktop emulator (only injected on localhost): start in hand-tracking mode, the app ignores controllers
  emulate: { primaryInputMode: 'hand' },
  foveation: 1,
  frameRate: 'high',
})

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
    })

    const enter = () => xrStore.enterVR()
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
    select.value = TASKS[params.get('task')] ? params.get('task') : DEFAULT_TASK
    const onChange = () => {
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
    const unsubscribe = onEpisodesChanged(({ total, success }) => {
      count.textContent = total ? `${total} episodes recorded (${success} successful)` : 'No episodes recorded yet'
      download.disabled = clear.disabled = total === 0
    })
    const onDownload = async () => {
      const url = URL.createObjectURL(await exportEpisodes())
      const a = document.createElement('a')
      a.href = url
      a.download = `iamr_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.iamr`
      a.click()
      setTimeout(() => URL.revokeObjectURL(url), 10000)
    }
    const onClear = () => { if (confirm('Delete all recorded episodes from this headset?')) clearEpisodes() }
    download.addEventListener('click', onDownload)
    clear.addEventListener('click', onClear)
    return () => {
      unsubscribe()
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
