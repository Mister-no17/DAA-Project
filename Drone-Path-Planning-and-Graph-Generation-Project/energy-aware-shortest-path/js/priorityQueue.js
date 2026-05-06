export class MinPriorityQueue {
  constructor() {
    this.heap = [];
    this.counter = 0;
  }

  get size() {
    return this.heap.length;
  }

  push(priority, value) {
    const node = {
      priority,
      value,
      order: this.counter,
    };

    this.counter += 1;
    this.heap.push(node);
    this.bubbleUp(this.heap.length - 1);
  }

  pop() {
    if (this.heap.length === 0) {
      return null;
    }

    const min = this.heap[0];
    const end = this.heap.pop();

    if (this.heap.length > 0) {
      this.heap[0] = end;
      this.bubbleDown(0);
    }

    return min;
  }

  bubbleUp(index) {
    let child = index;

    while (child > 0) {
      const parent = Math.floor((child - 1) / 2);

      if (this.compare(this.heap[child], this.heap[parent])) {
        [this.heap[child], this.heap[parent]] = [this.heap[parent], this.heap[child]];
        child = parent;
      } else {
        break;
      }
    }
  }

  bubbleDown(index) {
    let parent = index;
    const length = this.heap.length;

    while (true) {
      const left = parent * 2 + 1;
      const right = parent * 2 + 2;
      let smallest = parent;

      if (left < length && this.compare(this.heap[left], this.heap[smallest])) {
        smallest = left;
      }

      if (right < length && this.compare(this.heap[right], this.heap[smallest])) {
        smallest = right;
      }

      if (smallest === parent) {
        break;
      }

      [this.heap[parent], this.heap[smallest]] = [this.heap[smallest], this.heap[parent]];
      parent = smallest;
    }
  }

  compare(a, b) {
    if (a.priority !== b.priority) {
      return a.priority < b.priority;
    }

    return a.order < b.order;
  }
}
