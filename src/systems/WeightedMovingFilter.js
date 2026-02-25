/**
 * Weighted moving average over the last N frames.
 * Ported from xr_teleoperate. Most recent frame gets highest weight.
 */
export class WeightedMovingFilter {
  constructor(weights, dataSize) {
    this.weights = weights
    this.windowSize = weights.length
    this.dataSize = dataSize
    this.filteredData = new Float64Array(dataSize)
    this.queue = []
  }

  addData(newData) {
    if (this.queue.length >= this.windowSize) this.queue.shift()
    this.queue.push(Float64Array.from(newData))

    if (this.queue.length < this.windowSize) {
      this.filteredData.set(this.queue[this.queue.length - 1])
      return this.filteredData
    }

    for (let j = 0; j < this.dataSize; j++) {
      let sum = 0
      for (let i = 0; i < this.windowSize; i++) {
        sum += this.queue[this.queue.length - 1 - i][j] * this.weights[i]
      }
      this.filteredData[j] = sum
    }
    return this.filteredData
  }

  reset() {
    this.queue = []
    this.filteredData.fill(0)
  }
}
