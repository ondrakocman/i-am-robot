import { execSync } from 'node:child_process'
import { readFileSync } from 'node:fs'
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import basicSsl from '@vitejs/plugin-basic-ssl'

const mujocoVersion = JSON.parse(readFileSync(new URL('./node_modules/@mujoco/mujoco/package.json', import.meta.url))).version
let gitSha = 'unknown'
try { gitSha = execSync('git rev-parse --short HEAD', { stdio: ['ignore', 'pipe', 'ignore'] }).toString().trim() } catch { /* not a checkout */ }

// Deployed under /<repo>/ on GitHub Pages; override with BASE_PATH for any other host
const basePath = process.env.BASE_PATH ?? (process.env.GITHUB_REPOSITORY ? `/${process.env.GITHUB_REPOSITORY.split('/')[1]}/` : '/i-am-robot/')

export default defineConfig(({ command }) => ({
  plugins: [
    react(),
    ...(command === 'serve' ? [basicSsl()] : []), // WebXR needs https, also on the LAN
  ],
  base: command === 'build' ? basePath : '/',
  define: {
    __MUJOCO_VERSION__: JSON.stringify(mujocoVersion),
    __GIT_SHA__: JSON.stringify(gitSha),
  },
  // MuJoCo's emscripten glue finds mujoco.wasm via import.meta.url; pre-bundling would break that
  optimizeDeps: { exclude: ['@mujoco/mujoco'] },
  worker: { format: 'es' },
  build: { chunkSizeWarningLimit: 2200 }, // the XR emulator's room models (lazy, localhost only) are ~2 MB
  server: {
    https: true,
    host: '0.0.0.0',
    port: 5173,
  },
}))
