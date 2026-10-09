// Printed shipping label for the conveyor packages: address block, barcode and a QR-like code, drawn into a
// canvas once per seed and shared by every package that asks for it.
import * as THREE from 'three'

// ── Shipping label texture: address block, barcode and QR-like code, deterministic per seed ──────────

const LABEL_NAMES = ['A. Novák', 'M. Dvořák', 'J. Svoboda', 'K. Černá', 'P. Procházka', 'L. Kučera', 'T. Veselý', 'E. Horák']
const LABEL_STREETS = ['Vinohradská 12', 'Masarykova 8', 'Dlouhá 3', 'Nádražní 41', 'Lipová 27', 'Krátká 5', 'Jiráskova 19', 'Polní 2']
const labelTextures = new Map()

export function makeLabelTexture(seed) {
  if (labelTextures.has(seed)) return labelTextures.get(seed)
  let a = (seed * 2654435761 + 12345) >>> 0
  const rnd = () => { a = (a * 1664525 + 1013904223) >>> 0; return a / 4294967296 }
  for (let i = 0; i < 4; i++) rnd() // the first draws of a small-seed LCG are nearly the same for every seed
  const canvas = document.createElement('canvas')
  canvas.width = 512
  canvas.height = 352
  const ctx = canvas.getContext('2d')
  ctx.fillStyle = '#f6f5f0'
  ctx.fillRect(0, 0, 512, 352)
  ctx.fillStyle = '#111'
  ctx.font = '700 30px system-ui, sans-serif'
  ctx.fillText('SHIP TO', 28, 46)
  ctx.font = '400 26px system-ui, sans-serif'
  ctx.fillText(LABEL_NAMES[Math.floor(rnd() * LABEL_NAMES.length)], 28, 86)
  ctx.fillText(LABEL_STREETS[Math.floor(rnd() * LABEL_STREETS.length)], 28, 118)
  ctx.fillText(`${110 + Math.floor(rnd() * 600)} 00 Praha ${1 + Math.floor(rnd() * 10)}`, 28, 150)
  ctx.font = '700 22px system-ui, sans-serif'
  ctx.fillText(rnd() < 0.5 ? 'PRIORITY' : 'STANDARD', 28, 190)
  ctx.strokeStyle = '#111'
  ctx.lineWidth = 3
  ctx.strokeRect(14, 14, 484, 324)
  // barcode: random bar widths, with the number printed under it
  let x = 28
  const digits = String(Math.floor(rnd() * 1e9)).padStart(9, '0')
  while (x < 330) {
    const w = 2 + Math.floor(rnd() * 4)
    if (rnd() < 0.55) ctx.fillRect(x, 220, w, 80)
    x += w + 1
  }
  ctx.font = '400 20px monospace'
  ctx.fillText(`PKG ${digits.slice(0, 3)} ${digits.slice(3, 6)} ${digits.slice(6)}`, 28, 326)
  // QR-like code: 17x17 random modules with the three finder patterns
  const n = 17, cell = 7, ox = 372, oy = 196
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
    const finder = (i < 7 && j < 7) || (i < 7 && j >= n - 7) || (i >= n - 7 && j < 7)
    const on = finder ? ((i % 6 === 0 || j % 6 === 0 || (i >= 2 && i <= 4 && j >= 2 && j <= 4)) ^ (i >= n - 7) ^ (j >= n - 7)) : rnd() < 0.45
    if (on) ctx.fillRect(ox + j * cell, oy + i * cell, cell, cell)
  }
  const texture = new THREE.CanvasTexture(canvas)
  texture.colorSpace = THREE.SRGBColorSpace
  texture.anisotropy = 4
  labelTextures.set(seed, texture)
  return texture
}
