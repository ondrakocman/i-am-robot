// Converts all STL files to GLB with indexed geometry (vertex dedup)
// Reduces ~30MB of unindexed STL to ~5-8MB of indexed GLB

import * as THREE from 'three'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import { mergeVertices } from 'three/examples/jsm/utils/BufferGeometryUtils.js'
import fs from 'fs'
import path from 'path'

const MESHES_DIR = path.resolve('public/models/meshes')

function encodeIndexedGLB(geometry) {
  geometry.computeVertexNormals()

  // Merge duplicate vertices (STL stores 3 unique verts per face)
  const indexed = mergeVertices(geometry, 1e-4)
  indexed.computeVertexNormals()

  const position = indexed.getAttribute('position')
  const normal = indexed.getAttribute('normal')
  const index = indexed.getIndex()
  const vertexCount = position.count
  const indexCount = index.count

  // Compute bounds
  indexed.computeBoundingBox()
  const bb = indexed.boundingBox
  const posMin = [bb.min.x, bb.min.y, bb.min.z]
  const posMax = [bb.max.x, bb.max.y, bb.max.z]

  // Vertex buffer: position (12B) + normal (12B) = 24B per vertex
  const vertexBytes = vertexCount * 24
  const vertexBuf = Buffer.alloc(vertexBytes)
  for (let i = 0; i < vertexCount; i++) {
    const off = i * 24
    vertexBuf.writeFloatLE(position.getX(i), off)
    vertexBuf.writeFloatLE(position.getY(i), off + 4)
    vertexBuf.writeFloatLE(position.getZ(i), off + 8)
    vertexBuf.writeFloatLE(normal.getX(i), off + 12)
    vertexBuf.writeFloatLE(normal.getY(i), off + 16)
    vertexBuf.writeFloatLE(normal.getZ(i), off + 20)
  }

  // Index buffer: uint16 if < 65536 verts, else uint32
  const use16 = vertexCount < 65536
  const indexByteSize = use16 ? 2 : 4
  const indexBytes = indexCount * indexByteSize
  const indexBuf = Buffer.alloc(indexBytes)
  for (let i = 0; i < indexCount; i++) {
    if (use16) indexBuf.writeUInt16LE(index.getX(i), i * 2)
    else indexBuf.writeUInt32LE(index.getX(i), i * 4)
  }

  // Pad buffers to 4-byte alignment
  const padTo4 = (n) => (n + 3) & ~3
  const vertexPadded = padTo4(vertexBytes)
  const indexPadded = padTo4(indexBytes)
  const totalBinLength = vertexPadded + indexPadded

  const binBuf = Buffer.alloc(totalBinLength, 0)
  vertexBuf.copy(binBuf, 0)
  indexBuf.copy(binBuf, vertexPadded)

  const gltf = {
    asset: { version: '2.0', generator: 'i-am-robot' },
    scene: 0,
    scenes: [{ nodes: [0] }],
    nodes: [{ mesh: 0 }],
    meshes: [{
      primitives: [{
        attributes: { POSITION: 0, NORMAL: 1 },
        indices: 2,
        material: 0,
      }],
    }],
    materials: [{
      pbrMetallicRoughness: {
        baseColorFactor: [0.102, 0.102, 0.176, 1.0],
        metallicFactor: 0.3,
        roughnessFactor: 0.5,
      },
    }],
    accessors: [
      { bufferView: 0, byteOffset: 0, componentType: 5126, count: vertexCount, type: 'VEC3', min: posMin, max: posMax },
      { bufferView: 0, byteOffset: 12, componentType: 5126, count: vertexCount, type: 'VEC3' },
      { bufferView: 1, byteOffset: 0, componentType: use16 ? 5123 : 5125, count: indexCount, type: 'SCALAR' },
    ],
    bufferViews: [
      { buffer: 0, byteOffset: 0, byteLength: vertexBytes, byteStride: 24 },
      { buffer: 0, byteOffset: vertexPadded, byteLength: indexBytes },
    ],
    buffers: [{ byteLength: totalBinLength }],
  }

  const jsonStr = JSON.stringify(gltf)
  const jsonBuf = Buffer.from(jsonStr)
  const jsonPadLen = padTo4(jsonBuf.length)
  const jsonPad = Buffer.alloc(jsonPadLen, 0x20)
  jsonBuf.copy(jsonPad)

  const totalLength = 12 + 8 + jsonPadLen + 8 + totalBinLength
  const glb = Buffer.alloc(totalLength)
  let o = 0

  glb.writeUInt32LE(0x46546C67, o); o += 4
  glb.writeUInt32LE(2, o); o += 4
  glb.writeUInt32LE(totalLength, o); o += 4

  glb.writeUInt32LE(jsonPadLen, o); o += 4
  glb.writeUInt32LE(0x4E4F534A, o); o += 4
  jsonPad.copy(glb, o); o += jsonPadLen

  glb.writeUInt32LE(totalBinLength, o); o += 4
  glb.writeUInt32LE(0x004E4942, o); o += 4
  binBuf.copy(glb, o)

  return glb
}

async function main() {
  const files = fs.readdirSync(MESHES_DIR).filter(f => /\.STL$/i.test(f))
  console.log(`Converting ${files.length} STL files...\n`)

  const loader = new STLLoader()
  let totalStl = 0, totalGlb = 0

  for (const file of files) {
    const stlPath = path.join(MESHES_DIR, file)
    const stlSize = fs.statSync(stlPath).size
    totalStl += stlSize

    const data = fs.readFileSync(stlPath)
    const buf = data.buffer.slice(data.byteOffset, data.byteOffset + data.byteLength)
    const geom = loader.parse(buf)

    const glb = encodeIndexedGLB(geom)
    const glbPath = stlPath.replace(/\.STL$/i, '.glb')
    fs.writeFileSync(glbPath, glb)

    totalGlb += glb.length
    const ratio = ((1 - glb.length / stlSize) * 100).toFixed(0)
    console.log(`  ${file} (${(stlSize/1024).toFixed(0)}K → ${(glb.length/1024).toFixed(0)}K, -${ratio}%)`)
  }

  for (const file of files) {
    fs.unlinkSync(path.join(MESHES_DIR, file))
  }

  console.log(`\nTotal: ${(totalStl/1024/1024).toFixed(1)}MB → ${(totalGlb/1024/1024).toFixed(1)}MB`)
}

main().catch(console.error)
