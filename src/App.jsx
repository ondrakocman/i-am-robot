import { useEffect, useState } from 'react'
import { Canvas } from '@react-three/fiber'
import { createXRStore, XR } from '@react-three/xr'
import * as THREE from 'three'
import { Scene } from './components/Scene.jsx'

const xrStore = createXRStore({
  hand: { model: false },
  controller: false,
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

  return (
    <Canvas
      style={{ position: 'fixed', inset: 0 }}
      gl={{
        antialias: true,
        alpha: false,
        powerPreference: 'high-performance',
        toneMapping: THREE.NoToneMapping,
      }}
      camera={{
        fov: 75,
        near: 0.01,
        far: 100,
        position: [0, 1.24, 2],
      }}
    >
      <color attach="background" args={['#607080']} />
      <XR store={xrStore}>
        <Scene vrMode={vrMode} />
      </XR>
    </Canvas>
  )
}
