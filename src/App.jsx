import { useEffect, useRef } from 'react'
import { Canvas } from '@react-three/fiber'
import { createXRStore, XR } from '@react-three/xr'
import { Scene } from './components/Scene.jsx'

// ─── XR Store ─────────────────────────────────────────────────────────────────
// Disable built-in hand models — we render Dex 3.1 meshes instead
const xrStore = createXRStore({
  hand: {
    model: false,
  },
  controller: false,
})

// ─── App ──────────────────────────────────────────────────────────────────────
export default function App() {
  const btnRef = useRef()

  // Wire the HTML button to the XR store
  useEffect(() => {
    const btn = document.getElementById('enter-vr-btn')
    const status = document.getElementById('status')
    const instructions = document.querySelector('#instructions')

    if (!btn) return

    // Check WebXR support
    if (!navigator.xr) {
      btn.textContent = 'WebXR Not Available'
      btn.disabled = true
      if (status) status.textContent = 'Use Meta Quest 3 Browser or Chrome with WebXR flag'
      return
    }

    navigator.xr.isSessionSupported('immersive-vr').then((supported) => {
      if (!supported) {
        btn.textContent = 'VR Not Supported'
        btn.style.opacity = '0.5'
        if (status) status.textContent = 'Open this page on Meta Quest 3 browser'
      } else {
        if (status) status.textContent = 'Quest 3 Ready · Hand Tracking Required'
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
      }}
      camera={{
        fov: 75,
        near: 0.01,
        far: 100,
        position: [0, 1.24, 2],
      }}
      shadows={false}
    >
      <XR store={xrStore}>
        <Scene />
      </XR>
    </Canvas>
  )
}
