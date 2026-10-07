import { useEffect, useState } from 'react'
import { Canvas } from '@react-three/fiber'
import { createXRStore, XR } from '@react-three/xr'
import * as THREE from 'three'
import { Scene } from './components/Scene.jsx'
import { MujocoScene } from './components/MujocoScene.jsx'
import { onEpisodesChanged, exportEpisodes, clearEpisodes } from './sim/episodeStore.js'

// ?legacy = the original kinematic (Rapier) scene without the MuJoCo task
const LEGACY = new URLSearchParams(location.search).has('legacy')

const xrStore = createXRStore({
  hand: { model: false },
  controller: false,
  // Desktop emulator (only injected on localhost): start in hand-tracking mode, the app ignores controllers
  emulate: { primaryInputMode: 'hand' },
  foveation: 1,
  frameRate: 'high',
})

export default function App() {
  const [vrMode, setVrMode] = useState('unlocked')

  useEffect(() => {
    const lockedBtn = document.getElementById('enter-vr-locked')
    const unlockedBtn = document.getElementById('enter-vr-unlocked')
    const status = document.getElementById('status')
    const instructions = document.querySelector('#instructions')

    if (!lockedBtn || !unlockedBtn) return

    if (!navigator.xr) {
      lockedBtn.textContent = 'WebXR N/A'
      unlockedBtn.textContent = 'WebXR N/A'
      lockedBtn.disabled = true
      unlockedBtn.disabled = true
      if (status) status.textContent = 'Use Meta Quest 3 Browser'
      return
    }

    navigator.xr.isSessionSupported('immersive-vr').then((supported) => {
      if (!supported) {
        lockedBtn.style.opacity = '0.5'
        unlockedBtn.style.opacity = '0.5'
        if (status) status.textContent = 'Open on Meta Quest 3 browser'
      } else {
        if (status) status.textContent = 'Quest 3 Ready'
        if (instructions) instructions.style.display = 'none'
      }
    })

    const enterLocked = () => { setVrMode('locked'); xrStore.enterVR() }
    const enterUnlocked = () => { setVrMode('unlocked'); xrStore.enterVR() }

    lockedBtn.addEventListener('click', enterLocked)
    unlockedBtn.addEventListener('click', enterUnlocked)
    return () => {
      lockedBtn.removeEventListener('click', enterLocked)
      unlockedBtn.removeEventListener('click', enterUnlocked)
    }
  }, [])

  // Task selector: the task is a URL parameter so the worker and a reload agree on it
  useEffect(() => {
    const select = document.getElementById('task')
    if (!select) return
    const params = new URLSearchParams(location.search)
    if (params.get('task')) select.value = params.get('task')
    const onChange = () => {
      params.set('task', select.value)
      location.search = params.toString()
    }
    select.addEventListener('change', onChange)
    return () => select.removeEventListener('change', onChange)
  }, [])

  useEffect(() => {
    const row = document.getElementById('episodes')
    if (LEGACY || !row) return
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
      a.download = `tube_box_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.iamr`
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
        toneMapping: LEGACY ? THREE.NoToneMapping : THREE.ACESFilmicToneMapping,
      }}
      shadows={LEGACY ? false : 'percentage'}
      camera={{
        fov: 75,
        near: 0.01,
        far: 100,
        position: [0, 1.24, 2],
      }}
    >
      <color attach="background" args={['#607080']} />
      <XR store={xrStore}>
        {LEGACY ? <Scene vrMode={vrMode} /> : <MujocoScene vrMode={vrMode} />}
      </XR>
    </Canvas>
  )
}
