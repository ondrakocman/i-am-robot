// Scripted pick-and-place for headless task checks and desktop demos. Produces the same operator input the
// headset would: a palm pose + finger closure per hand.
//
// Only a sideways approach with a level palm is scripted. It works for round objects (the object rolls
// between thumb and fingers as the hand closes) but not for boxes: the open Dex3 thumb sticks straight out
// sideways and its tip hits a flat face before the fingers reach it, and a pitched-down palm is 2-4 cm off
// everywhere on the table for this arm. Tasks with boxes verify their goal logic with `solved()` instead.

const smooth = (a, b, u) => { u = Math.max(0, Math.min(1, u)); u = u * u * (3 - 2 * u); return a + (b - a) * u }

// Grip site relative to the palm site (x forward, y toward the fingers' closing side)
const GRIP_OFFSET = [[0.0735, -0.038], [0.0735, 0.038]]

/**
 * Keyframes for one hand picking an object at `from` and releasing it with the grip site at `to`, as
 * [time, grip x, y, z, palm yaw, finger closure]. The hand approaches from its own side (+y for the left
 * hand, -y for the right), lifts, carries with the palm turned inward, lowers and lets go, then returns
 * to `rest`. A held object sits a few cm behind and inside the grip site, so `to` is the grip target, not
 * the object's landing spot. Times are relative to `t0`.
 */
export function pickPlaceKeys(side, rest, from, to, { t0 = 0, lift = 0.13, carryYaw = 0.5, dropHeight = 0.02 } = {}) {
  const sgn = side === 0 ? 1 : -1
  const [fx, fy, fz] = from
  const [tx, ty, tz] = to
  const yaw = -sgn * carryYaw
  const k = (t, x, y, z, yw, close) => [t0 + t, x, y, z, yw, close]
  // The hand settles for a second before closing: a moving hand shoves light objects away
  return [
    k(0, rest[0], rest[1], rest[2], 0, 0),
    k(1.0, fx, fy + sgn * 0.09, fz + lift, 0, 0),
    k(2.0, fx, fy + sgn * 0.09, fz, 0, 0),
    k(3.0, fx, fy + sgn * 0.012, fz, 0, 0),
    k(4.2, fx, fy + sgn * 0.012, fz, 0, 0),
    k(5.0, fx, fy + sgn * 0.012, fz, 0, 1),
    k(6.0, fx, fy + sgn * 0.012, fz + lift, 0, 1),
    k(8.0, tx, ty, tz + lift, yaw, 1),
    k(9.0, tx, ty, tz + dropHeight, yaw, 1),
    k(9.6, tx, ty, tz + dropHeight, yaw, 0),
    k(11.0, rest[0], rest[1], rest[2], 0, 0),
  ]
}

export const PICK_PLACE_DURATION = 11

/** Interpolates a keyframe list at time t -> [gx, gy, gz, yaw, close]. */
export function sampleKeys(keys, t) {
  let i = 0
  while (i < keys.length - 2 && t > keys[i + 1][0]) i++
  const [t0, ...a] = keys[i]
  const [t1, ...b] = keys[i + 1]
  const u = t1 > t0 ? (t - t0) / (t1 - t0) : 1
  return a.map((v, k) => smooth(v, b[k], u))
}

/** Writes one hand's operator input (tracked, palm pose, fingers) for a grip-site target. */
export function writeHandInput(input, offset, side, [gx, gy, gz, yaw, close]) {
  const c = Math.cos(yaw), s = Math.sin(yaw)
  const [ox, oy] = GRIP_OFFSET[side]
  input[offset] = 1
  // palm = grip - R(yaw) * gripOffset; palm frame = world rotated by yaw about z
  input[offset + 1] = gx - (c * ox - s * oy)
  input[offset + 2] = gy - (s * ox + c * oy)
  input[offset + 3] = gz
  input[offset + 4] = Math.cos(yaw / 2); input[offset + 5] = 0; input[offset + 6] = 0; input[offset + 7] = Math.sin(yaw / 2)
  input[offset + 8] = 0
  for (let k = 1; k < 7; k++) input[offset + 8 + k] = close
}
