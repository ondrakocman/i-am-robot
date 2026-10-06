import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import basicSsl from '@vitejs/plugin-basic-ssl'
import { readFileSync } from 'node:fs'

const mujocoVersion = JSON.parse(readFileSync(new URL('./node_modules/@mujoco/mujoco/package.json', import.meta.url))).version

export default defineConfig(({ command }) => ({
  plugins: [
    react(),
    ...(command === 'serve' ? [basicSsl()] : []),
  ],
  base: command === 'build' ? '/i-am-robot/' : '/',
  define: { __MUJOCO_VERSION__: JSON.stringify(mujocoVersion) },
  // MuJoCo's emscripten glue finds mujoco.wasm via import.meta.url; pre-bundling would break that
  optimizeDeps: { exclude: ['@mujoco/mujoco'] },
  worker: { format: 'es' },
  server: {
    https: true,
    host: '0.0.0.0',
    port: 5173,
  },
}))
