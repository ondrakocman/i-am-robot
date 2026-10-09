import js from '@eslint/js'
import globals from 'globals'
import reactHooks from 'eslint-plugin-react-hooks'

const base = {
  languageOptions: {
    ecmaVersion: 2024,
    sourceType: 'module',
    parserOptions: { ecmaFeatures: { jsx: true } },
  },
  rules: {
    'no-unused-vars': ['error', { argsIgnorePattern: '^_', varsIgnorePattern: '^_|^React$' }],
  },
}

export default [
  { ignores: ['dist/', 'node_modules/', 'scratch/'] },
  js.configs.recommended,
  // physics and task code: environment-agnostic (worker and Node); crypto/performance are in both
  {
    files: ['src/sim/**/*.js'],
    ignores: ['src/sim/episodeStore.js'],
    ...base,
    languageOptions: { ...base.languageOptions, globals: { ...globals['shared-node-browser'], __MUJOCO_VERSION__: 'readonly' } },
  },
  { files: ['src/sim/sim.worker.js'], languageOptions: { globals: { ...globals.worker, __MUJOCO_VERSION__: 'readonly' } } },
  // browser UI
  {
    files: ['src/**/*.jsx', 'src/components/**/*.js', 'src/systems/**/*.js', 'src/constants/**/*.js', 'src/sim/episodeStore.js'],
    ...base,
    languageOptions: { ...base.languageOptions, globals: { ...globals.browser, __GIT_SHA__: 'readonly' } },
    plugins: { 'react-hooks': reactHooks },
    rules: { ...base.rules, ...reactHooks.configs.recommended.rules },
  },
  // tooling
  {
    files: ['scripts/**/*.mjs', 'vite.config.js', 'eslint.config.js'],
    ...base,
    languageOptions: { ...base.languageOptions, globals: globals.node },
  },
]
