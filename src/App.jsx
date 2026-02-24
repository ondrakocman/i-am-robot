import { useEffect } from 'react'
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
  useEffect(() => {
    const btn = document.getElementById('enter-vr-btn')
    const status = document.getElementById('status')
    const instructions = document.querySelector('#instructions')

    if (!btn) return

    if (!navigator.xr) {
      btn.textContent = 'WebXR Not Available'
      btn.disabled = true
      if (status) status.textContent = 'Use Meta Quest 3 Browser'
      return
    }

    navigator.xr.isSessionSupported('immersive-vr').then((supported) => {
      if (!supported) {
        btn.textContent = 'VR Not Supported'
        btn.style.opacity = '0.5'
        if (status) status.textContent = 'Open on Meta Quest 3 browser'
      } else {
        if (status) status.textContent = 'Quest 3 Ready'
        if (instructions) instructions.style.display = 'none'
      }
    })

    const handleClick = () => xrStore.enterVR()
    btn.addEventListener('click', handleClick)
    return () => btn.removeEventListener('click', handleClick)
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
        <Scene />
      </XR>
    </Canvas>
  )
}
