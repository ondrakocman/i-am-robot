export class ExponentialSmoother {
  constructor(alpha = 0.08) {
    this.alpha = alpha
    this.value = null
  }

  update(target) {
    if (this.value === null) {
      this.value = target.clone()
      return this.value
    }
    this.value.lerp(target, this.alpha)
    return this.value
  }

  reset(value) {
    this.value = value ? value.clone() : null
  }
}

export class QuaternionSmoother {
  constructor(alpha = 0.12) {
    this.alpha = alpha
    this.value = null
  }

  update(target) {
    if (this.value === null) {
      this.value = target.clone()
      return this.value
    }
    this.value.slerp(target, this.alpha)
    return this.value
  }

  reset(value) {
    this.value = value ? value.clone() : null
  }
}
