import { MinPriorityQueue } from "./priorityQueue.js";

export const DIRECTION_VECTORS = Object.freeze({
  N: [-1, 0],
  NE: [-Math.SQRT1_2, Math.SQRT1_2],
  E: [0, 1],
  SE: [Math.SQRT1_2, Math.SQRT1_2],
  S: [1, 0],
  SW: [Math.SQRT1_2, -Math.SQRT1_2],
  W: [0, -1],
  NW: [-Math.SQRT1_2, -Math.SQRT1_2],
});

export const DEFAULT_ENVIRONMENT = Object.freeze({
  windDirection: "E",
  windStrength: 1.5,
  altitudeFactor: 1.0,
  downhillFactor: 0.35,
  dynamicWindEnabled: false,
  windShiftStep: 6,
  windDirectionAfterShift: "W",
  windStrengthAfterShift: 2.2,
  maxStepMultiplier: 2.5,
});

const NEIGHBOR_DELTAS = [
  [1, 0],
  [0, 1],
  [-1, 0],
  [0, -1],
];

function toIndex(row, col, cols) {
  return row * cols + col;
}

function toCoord(index, cols) {
  return [Math.floor(index / cols), index % cols];
}

function inBounds(row, col, rows, cols) {
  return row >= 0 && row < rows && col >= 0 && col < cols;
}

function neighbors(row, col, rows, cols) {
  const output = [];

  for (const [dr, dc] of NEIGHBOR_DELTAS) {
    const nr = row + dr;
    const nc = col + dc;

    if (inBounds(nr, nc, rows, cols)) {
      output.push([nr, nc]);
    }
  }

  return output;
}

function isBlockedCell(row, col, blockedSet) {
  if (!blockedSet || blockedSet.size === 0) {
    return false;
  }

  return blockedSet.has(`${row},${col}`);
}

function reconstructPath(previous, startIndex, endIndex, cols) {
  const path = [];
  let current = endIndex;

  while (current !== -1) {
    path.push(toCoord(current, cols));

    if (current === startIndex) {
      break;
    }

    current = previous[current];
  }

  path.reverse();

  if (path.length === 0) {
    return [];
  }

  const [startRow, startCol] = toCoord(startIndex, cols);
  if (path[0][0] !== startRow || path[0][1] !== startCol) {
    return [];
  }

  return path;
}

export function geometricDistance(from, to) {
  return Math.hypot(to[0] - from[0], to[1] - from[1]);
}

export function getWindStateAtStep(step, environment) {
  if (!environment?.dynamicWindEnabled) {
    return {
      direction: environment.windDirection,
      strength: environment.windStrength,
    };
  }

  if (step < environment.windShiftStep) {
    return {
      direction: environment.windDirection,
      strength: environment.windStrength,
    };
  }

  return {
    direction: environment.windDirectionAfterShift,
    strength: environment.windStrengthAfterShift,
  };
}

export function computeWindEffect(from, to, windDirection, windStrength) {
  const windVector = DIRECTION_VECTORS[windDirection] ?? DIRECTION_VECTORS.E;
  const moveRow = to[0] - from[0];
  const moveCol = to[1] - from[1];
  const norm = Math.hypot(moveRow, moveCol) || 1;

  const unitMoveRow = moveRow / norm;
  const unitMoveCol = moveCol / norm;
  const dot = unitMoveRow * windVector[0] + unitMoveCol * windVector[1];

  return windStrength * (1 - dot);
}

export function computeAltitudeCost(from, to, altitudeGrid, altitudeFactor, downhillFactor = 0.35) {
  const currentAltitude = altitudeGrid[from[0]][from[1]];
  const nextAltitude = altitudeGrid[to[0]][to[1]];
  const climb = Math.max(0, nextAltitude - currentAltitude);
  const descent = Math.max(0, currentAltitude - nextAltitude);

  return altitudeFactor * (climb + downhillFactor * descent);
}

export function computeEnergyEdgeCost(from, to, altitudeGrid, environment) {
  const distance = geometricDistance(from, to);
  const windState = getWindStateAtStep(0, environment);
  const wind = computeWindEffect(from, to, windState.direction, windState.strength);
  const altitude = computeAltitudeCost(
    from,
    to,
    altitudeGrid,
    environment.altitudeFactor,
    environment.downhillFactor,
  );

  return distance + wind + altitude;
}

export function computeEnergyEdgeCostAtStep(from, to, altitudeGrid, environment, step = 0) {
  const distance = geometricDistance(from, to);
  const windState = getWindStateAtStep(step, environment);
  const wind = computeWindEffect(from, to, windState.direction, windState.strength);
  const altitude = computeAltitudeCost(
    from,
    to,
    altitudeGrid,
    environment.altitudeFactor,
    environment.downhillFactor,
  );

  return distance + wind + altitude;
}

export function dijkstraGrid({ rows, cols, start, end, weightFn, blockedSet }) {
  const nodeCount = rows * cols;
  const distances = new Array(nodeCount).fill(Number.POSITIVE_INFINITY);
  const previous = new Array(nodeCount).fill(-1);
  const visited = new Array(nodeCount).fill(false);

  const startIndex = toIndex(start[0], start[1], cols);
  const endIndex = toIndex(end[0], end[1], cols);

  if (isBlockedCell(start[0], start[1], blockedSet) || isBlockedCell(end[0], end[1], blockedSet)) {
    return {
      path: [],
      objectiveCost: Number.POSITIVE_INFINITY,
      expandedNodes: 0,
    };
  }

  const queue = new MinPriorityQueue();
  distances[startIndex] = 0;
  queue.push(0, startIndex);

  while (queue.size > 0) {
    const currentNode = queue.pop();

    if (currentNode === null) {
      break;
    }

    const currentIndex = currentNode.value;
    const currentCost = currentNode.priority;

    if (visited[currentIndex]) {
      continue;
    }

    visited[currentIndex] = true;

    if (currentIndex === endIndex) {
      break;
    }

    const [row, col] = toCoord(currentIndex, cols);

    for (const [nextRow, nextCol] of neighbors(row, col, rows, cols)) {
      if (isBlockedCell(nextRow, nextCol, blockedSet)) {
        continue;
      }

      const nextIndex = toIndex(nextRow, nextCol, cols);

      if (visited[nextIndex]) {
        continue;
      }

      const edgeCost = weightFn([row, col], [nextRow, nextCol]);
      if (!Number.isFinite(edgeCost) || edgeCost < 0) {
        continue;
      }

      const candidateCost = currentCost + edgeCost;

      if (candidateCost < distances[nextIndex]) {
        distances[nextIndex] = candidateCost;
        previous[nextIndex] = currentIndex;
        queue.push(candidateCost, nextIndex);
      }
    }
  }

  const hasPath = Number.isFinite(distances[endIndex]);
  const path = hasPath ? reconstructPath(previous, startIndex, endIndex, cols) : [];
  const expandedNodes = visited.reduce((count, seen) => count + (seen ? 1 : 0), 0);

  return {
    path,
    objectiveCost: distances[endIndex],
    expandedNodes,
  };
}

function timedStateKey(index, step) {
  return `${index}|${step}`;
}

function parseTimedStateKey(key) {
  const [index, step] = key.split("|").map(Number);
  return { index, step };
}

function reconstructTimedPath(previous, endKey, cols) {
  const reversed = [];
  let key = endKey;

  while (key) {
    const { index } = parseTimedStateKey(key);
    reversed.push(toCoord(index, cols));
    key = previous.get(key);
  }

  return reversed.reverse();
}

export function dijkstraGridTimeAware({ rows, cols, start, end, weightFn, blockedSet, maxSteps }) {
  const nodeCount = rows * cols;
  const startIndex = toIndex(start[0], start[1], cols);
  const endIndex = toIndex(end[0], end[1], cols);

  if (isBlockedCell(start[0], start[1], blockedSet) || isBlockedCell(end[0], end[1], blockedSet)) {
    return {
      path: [],
      objectiveCost: Number.POSITIVE_INFINITY,
      expandedNodes: 0,
      arrivalStep: -1,
    };
  }

  const horizon = Math.max(1, Math.floor(maxSteps));
  const distances = Array.from({ length: horizon + 1 }, () =>
    new Array(nodeCount).fill(Number.POSITIVE_INFINITY),
  );
  const previous = new Map();
  const settledStates = new Set();

  const queue = new MinPriorityQueue();
  distances[0][startIndex] = 0;
  queue.push(0, { index: startIndex, step: 0 });

  let bestEndKey = null;
  let expandedNodes = 0;

  while (queue.size > 0) {
    const currentNode = queue.pop();

    if (currentNode === null) {
      break;
    }

    const { index: currentIndex, step: currentStep } = currentNode.value;
    const stateKey = timedStateKey(currentIndex, currentStep);

    if (settledStates.has(stateKey)) {
      continue;
    }

    const knownDistance = distances[currentStep][currentIndex];
    if (currentNode.priority > knownDistance) {
      continue;
    }

    settledStates.add(stateKey);
    expandedNodes += 1;

    if (currentIndex === endIndex) {
      bestEndKey = stateKey;
      break;
    }

    if (currentStep >= horizon) {
      continue;
    }

    const [row, col] = toCoord(currentIndex, cols);

    for (const [nextRow, nextCol] of neighbors(row, col, rows, cols)) {
      if (isBlockedCell(nextRow, nextCol, blockedSet)) {
        continue;
      }

      const nextIndex = toIndex(nextRow, nextCol, cols);
      const edgeCost = weightFn([row, col], [nextRow, nextCol], currentStep);

      if (!Number.isFinite(edgeCost) || edgeCost < 0) {
        continue;
      }

      const nextStep = currentStep + 1;
      const candidate = knownDistance + edgeCost;

      if (candidate < distances[nextStep][nextIndex]) {
        distances[nextStep][nextIndex] = candidate;
        const nextKey = timedStateKey(nextIndex, nextStep);
        previous.set(nextKey, stateKey);
        queue.push(candidate, { index: nextIndex, step: nextStep });
      }
    }
  }

  if (!bestEndKey) {
    return {
      path: [],
      objectiveCost: Number.POSITIVE_INFINITY,
      expandedNodes,
      arrivalStep: -1,
    };
  }

  const { index: finalIndex, step: arrivalStep } = parseTimedStateKey(bestEndKey);
  const objectiveCost = distances[arrivalStep][finalIndex];
  const path = reconstructTimedPath(previous, bestEndKey, cols);

  return {
    path,
    objectiveCost,
    expandedNodes,
    arrivalStep,
  };
}

export function evaluatePathMetrics(path, altitudeGrid, environment) {
  if (!path || path.length === 0) {
    return {
      distance: Number.POSITIVE_INFINITY,
      energyCost: Number.POSITIVE_INFINITY,
      steps: 0,
      windPenalty: Number.POSITIVE_INFINITY,
      altitudePenalty: Number.POSITIVE_INFINITY,
    };
  }

  let distance = 0;
  let energyCost = 0;
  let windPenalty = 0;
  let altitudePenalty = 0;

  for (let i = 0; i < path.length - 1; i += 1) {
    const from = path[i];
    const to = path[i + 1];
    const edgeDistance = geometricDistance(from, to);
    const windState = getWindStateAtStep(i, environment);
    const wind = computeWindEffect(from, to, windState.direction, windState.strength);
    const altitude = computeAltitudeCost(
      from,
      to,
      altitudeGrid,
      environment.altitudeFactor,
      environment.downhillFactor,
    );
    distance += edgeDistance;
    windPenalty += wind;
    altitudePenalty += altitude;
    energyCost += edgeDistance + wind + altitude;
  }

  return {
    distance,
    energyCost,
    steps: Math.max(0, path.length - 1),
    windPenalty,
    altitudePenalty,
  };
}

export function runStandardDijkstra(model) {
  const rows = model.altitudeGrid.length;
  const cols = model.altitudeGrid[0].length;
  const startTime = performance.now();
  const blockedSet = model.blockedSet ?? new Set();

  const searchResult = dijkstraGrid({
    rows,
    cols,
    start: model.start,
    end: model.end,
    weightFn: geometricDistance,
    blockedSet,
  });

  const executionTimeMs = performance.now() - startTime;
  const metrics = evaluatePathMetrics(searchResult.path, model.altitudeGrid, model.environment);

  return {
    algorithm: "Standard Dijkstra",
    optimizedFor: "Distance",
    path: searchResult.path,
    objectiveCost: searchResult.objectiveCost,
    totalDistance: metrics.distance,
    totalEnergyCost: metrics.energyCost,
    steps: metrics.steps,
    windPenalty: metrics.windPenalty,
    altitudePenalty: metrics.altitudePenalty,
    blockedCells: blockedSet.size,
    expandedNodes: searchResult.expandedNodes,
    executionTimeMs,
    dynamicWindUsed: false,
  };
}

export function runModifiedDijkstra(model) {
  const rows = model.altitudeGrid.length;
  const cols = model.altitudeGrid[0].length;
  const startTime = performance.now();
  const blockedSet = model.blockedSet ?? new Set();

  const dynamicMode = model.environment.dynamicWindEnabled;

  let searchResult;

  if (dynamicMode) {
    const horizon = Math.max(
      rows + cols,
      Math.floor(rows * cols * (model.environment.maxStepMultiplier ?? 2.5)),
    );

    searchResult = dijkstraGridTimeAware({
      rows,
      cols,
      start: model.start,
      end: model.end,
      blockedSet,
      maxSteps: horizon,
      weightFn: (from, to, step) =>
        computeEnergyEdgeCostAtStep(from, to, model.altitudeGrid, model.environment, step),
    });
  } else {
    searchResult = dijkstraGrid({
      rows,
      cols,
      start: model.start,
      end: model.end,
      blockedSet,
      weightFn: (from, to) => computeEnergyEdgeCost(from, to, model.altitudeGrid, model.environment),
    });
  }

  const executionTimeMs = performance.now() - startTime;
  const metrics = evaluatePathMetrics(searchResult.path, model.altitudeGrid, model.environment);

  return {
    algorithm: "Modified Dijkstra",
    optimizedFor: dynamicMode
      ? "Distance + Wind + Altitude + Time Shift"
      : "Distance + Wind + Altitude",
    path: searchResult.path,
    objectiveCost: searchResult.objectiveCost,
    totalDistance: metrics.distance,
    totalEnergyCost: metrics.energyCost,
    steps: metrics.steps,
    windPenalty: metrics.windPenalty,
    altitudePenalty: metrics.altitudePenalty,
    blockedCells: blockedSet.size,
    expandedNodes: searchResult.expandedNodes,
    executionTimeMs,
    dynamicWindUsed: dynamicMode,
    arrivalStep: searchResult.arrivalStep ?? metrics.steps,
  };
}

function mulberry32(seed) {
  return function random() {
    let t = (seed += 0x6d2b79f5);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export function createRandomAltitudeGrid(rows, cols, seed = Math.floor(Math.random() * 2 ** 31)) {
  const random = mulberry32(seed >>> 0);
  const noise = Array.from({ length: rows }, () =>
    Array.from({ length: cols }, () => random()),
  );

  const smoothed = Array.from({ length: rows }, () => Array(cols).fill(0));

  for (let row = 0; row < rows; row += 1) {
    for (let col = 0; col < cols; col += 1) {
      let total = 0;
      let count = 0;

      for (let dr = -1; dr <= 1; dr += 1) {
        for (let dc = -1; dc <= 1; dc += 1) {
          const nr = row + dr;
          const nc = col + dc;

          if (inBounds(nr, nc, rows, cols)) {
            total += noise[nr][nc];
            count += 1;
          }
        }
      }

      const average = total / count;
      smoothed[row][col] = Number((average * 9).toFixed(1));
    }
  }

  return smoothed;
}

export function coordinateKey(coord) {
  return `${coord[0]},${coord[1]}`;
}

export function formatCoord(coord) {
  return `(${coord[0]},${coord[1]})`;
}
