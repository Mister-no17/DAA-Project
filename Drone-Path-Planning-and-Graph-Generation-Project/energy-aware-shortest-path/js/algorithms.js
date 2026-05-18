import { MinPriorityQueue } from "./priorityQueue.js";
import { computeEnergyComponents } from "./energy/totalEnergy.js";
import {
  energyAStarGrid,
  energyAStarGridTimeAware,
} from "./algorithms/energyAStar.js";

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
  distanceCoefficient: 1.0,
  averageEnergyFactor: 1.0,
  windCoefficient: 1.0,
  turningCoefficient: 0.0,
  mass: 1.0,
  gravity: 9.81,
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

function normalizeEnergyEnvironment(environment) {
  return {
    ...environment,
    distanceCoefficient: environment.distanceCoefficient ?? 1.0,
    windCoefficient: environment.windCoefficient ?? 1.0,
    turningCoefficient: environment.turningCoefficient ?? 0.0,
    mass: environment.mass ?? environment.altitudeFactor ?? 1.0,
    gravity: environment.gravity ?? 9.81,
  };
}

export function computeEnergyEdgeCost(from, to, altitudeGrid, environment) {
  const windState = getWindStateAtStep(0, environment);
  const energyEnv = normalizeEnergyEnvironment(environment);
  const components = computeEnergyComponents({
    from,
    to,
    prev: null,
    altitudeGrid,
    environment: energyEnv,
    windState,
    directionVectors: DIRECTION_VECTORS,
  });

  return components.totalEnergy;
}

export function computeEnergyEdgeCostAtStep(from, to, altitudeGrid, environment, step = 0) {
  const windState = getWindStateAtStep(step, environment);
  const energyEnv = normalizeEnergyEnvironment(environment);
  const components = computeEnergyComponents({
    from,
    to,
    prev: null,
    altitudeGrid,
    environment: energyEnv,
    windState,
    directionVectors: DIRECTION_VECTORS,
  });

  return components.totalEnergy;
}

export function dijkstraGrid({ rows, cols, start, end, weightFn, blockedSet }) {
  const nodeCount = rows * cols;
  const distances = new Array(nodeCount).fill(Number.POSITIVE_INFINITY);
  const previous = new Array(nodeCount).fill(-1);
  const visited = new Array(nodeCount).fill(false);
  let relaxationOperations = 0;

  const startIndex = toIndex(start[0], start[1], cols);
  const endIndex = toIndex(end[0], end[1], cols);

  if (isBlockedCell(start[0], start[1], blockedSet) || isBlockedCell(end[0], end[1], blockedSet)) {
    return {
      path: [],
      objectiveCost: Number.POSITIVE_INFINITY,
      expandedNodes: 0,
      uniqueVisitedNodes: 0,
      relaxationOperations: 0,
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

      relaxationOperations += 1;

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
    uniqueVisitedNodes: expandedNodes,
    relaxationOperations,
  };
}

export function bellmanFordGrid({ rows, cols, start, end, weightFn, blockedSet }) {
  const nodeCount = rows * cols;
  const distances = new Array(nodeCount).fill(Number.POSITIVE_INFINITY);
  const previous = new Array(nodeCount).fill(-1);
  const startIndex = toIndex(start[0], start[1], cols);
  const endIndex = toIndex(end[0], end[1], cols);
  const uniqueVisited = new Set();
  let relaxationOperations = 0;

  if (isBlockedCell(start[0], start[1], blockedSet) || isBlockedCell(end[0], end[1], blockedSet)) {
    return {
      path: [],
      objectiveCost: Number.POSITIVE_INFINITY,
      expandedNodes: 0,
      uniqueVisitedNodes: 0,
      relaxationOperations: 0,
    };
  }

  distances[startIndex] = 0;
  let expandedNodes = 0;
  uniqueVisited.add(startIndex);

  // Relax edges up to V-1 times
  for (let iter = 0; iter < nodeCount - 1; iter += 1) {
    let changed = false;

    for (let idx = 0; idx < nodeCount; idx += 1) {
      const [row, col] = toCoord(idx, cols);

      if (isBlockedCell(row, col, blockedSet)) {
        continue;
      }

      const distU = distances[idx];
      if (!Number.isFinite(distU)) {
        continue;
      }

      uniqueVisited.add(idx);

      for (const [nr, nc] of neighbors(row, col, rows, cols)) {
        if (isBlockedCell(nr, nc, blockedSet)) {
          continue;
        }

        const v = toIndex(nr, nc, cols);
        let edgeCost;
        try {
          edgeCost = weightFn([row, col], [nr, nc]);
        } catch (e) {
          // fallback if weightFn expects a step arg
          edgeCost = weightFn([row, col], [nr, nc], 0);
        }

        if (!Number.isFinite(edgeCost) || edgeCost < 0) {
          continue;
        }

        relaxationOperations += 1;
        const candidate = distU + edgeCost;

        if (candidate < distances[v]) {
          distances[v] = candidate;
          previous[v] = idx;
          changed = true;
          expandedNodes += 1;
        }
      }
    }

    if (!changed) {
      break;
    }
  }

  const hasPath = Number.isFinite(distances[endIndex]);
  const path = hasPath ? reconstructPath(previous, startIndex, endIndex, cols) : [];

  return {
    path,
    objectiveCost: distances[endIndex],
    expandedNodes,
    uniqueVisitedNodes: uniqueVisited.size,
    relaxationOperations,
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
      uniqueVisitedNodes: 0,
      relaxationOperations: 0,
    };
  }

  const horizon = Math.max(1, Math.floor(maxSteps));
  const distances = Array.from({ length: horizon + 1 }, () =>
    new Array(nodeCount).fill(Number.POSITIVE_INFINITY),
  );
  const previous = new Map();
  const settledStates = new Set();
  const uniqueVisited = new Set();
  let relaxationOperations = 0;

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
    uniqueVisited.add(currentIndex);

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

      relaxationOperations += 1;

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
      uniqueVisitedNodes: uniqueVisited.size,
      relaxationOperations,
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
    uniqueVisitedNodes: uniqueVisited.size,
    relaxationOperations,
  };
}

export function evaluatePathMetrics(path, altitudeGrid, environment) {
  if (!path || path.length === 0) {
    return {
      distance: Number.POSITIVE_INFINITY,
      energyCost: Number.POSITIVE_INFINITY,
      steps: 0,
      windEnergy: Number.POSITIVE_INFINITY,
      gravityEnergy: Number.POSITIVE_INFINITY,
      turningEnergy: Number.POSITIVE_INFINITY,
    };
  }

  let distance = 0;
  let energyCost = 0;
  let windEnergy = 0;
  let gravityEnergy = 0;
  let turningEnergy = 0;
  const energyEnv = normalizeEnergyEnvironment(environment);

  for (let i = 0; i < path.length - 1; i += 1) {
    const from = path[i];
    const to = path[i + 1];
    const prev = i > 0 ? path[i - 1] : null;
    const windState = getWindStateAtStep(i, environment);
    const components = computeEnergyComponents({
      from,
      to,
      prev,
      altitudeGrid,
      environment: energyEnv,
      windState,
      directionVectors: DIRECTION_VECTORS,
    });

    distance += components.distance;
    windEnergy += components.windEnergy;
    gravityEnergy += components.gravityEnergy;
    turningEnergy += components.turningEnergy;
    energyCost += components.totalEnergy;
  }

  return {
    distance,
    energyCost,
    steps: Math.max(0, path.length - 1),
    windEnergy,
    gravityEnergy,
    turningEnergy,
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
    optimizedFor: "Baseline distance",
    path: searchResult.path,
    objectiveCost: searchResult.objectiveCost,
    totalDistance: metrics.distance,
    totalEnergyCost: metrics.energyCost,
    steps: metrics.steps,
    windEnergy: metrics.windEnergy,
    gravityEnergy: metrics.gravityEnergy,
    turningEnergy: metrics.turningEnergy,
    blockedCells: blockedSet.size,
    expandedNodes: searchResult.expandedNodes,
    uniqueVisitedNodes: searchResult.uniqueVisitedNodes ?? searchResult.expandedNodes,
    relaxationOperations: searchResult.relaxationOperations ?? 0,
    executionTimeMs,
    dynamicWindUsed: false,
  };
}

export function runEnergyAwareAStar(model) {
  const rows = model.altitudeGrid.length;
  const cols = model.altitudeGrid[0].length;
  const startTime = performance.now();
  const blockedSet = model.blockedSet ?? new Set();

  const heuristicBase = model.environment.averageEnergyFactor ?? model.environment.distanceCoefficient ?? 1.0;
  const energyCoefficient = Math.max(0, heuristicBase);
  const heuristicFn = (coord) => energyCoefficient * geometricDistance(coord, model.end);

  const dynamicMode = model.environment.dynamicWindEnabled;

  let searchResult;

  if (dynamicMode) {
    const horizon = Math.max(
      rows + cols,
      Math.floor(rows * cols * (model.environment.maxStepMultiplier ?? 2.5)),
    );

    searchResult = energyAStarGridTimeAware({
      rows,
      cols,
      start: model.start,
      end: model.end,
      blockedSet,
      maxSteps: horizon,
      heuristicFn,
      weightFn: (from, to, step) =>
        computeEnergyEdgeCostAtStep(from, to, model.altitudeGrid, model.environment, step),
    });
  } else {
    searchResult = energyAStarGrid({
      rows,
      cols,
      start: model.start,
      end: model.end,
      blockedSet,
      heuristicFn,
      weightFn: (from, to) => computeEnergyEdgeCost(from, to, model.altitudeGrid, model.environment),
    });
  }

  const executionTimeMs = performance.now() - startTime;
  const metrics = evaluatePathMetrics(searchResult.path, model.altitudeGrid, model.environment);

  return {
    algorithm: "Energy-Aware A*",
    optimizedFor: dynamicMode
      ? "Minimum-energy trajectory (dynamic wind)"
      : "Minimum-energy trajectory",
    path: searchResult.path,
    objectiveCost: searchResult.objectiveCost,
    totalDistance: metrics.distance,
    totalEnergyCost: metrics.energyCost,
    steps: metrics.steps,
    windEnergy: metrics.windEnergy,
    gravityEnergy: metrics.gravityEnergy,
    turningEnergy: metrics.turningEnergy,
    blockedCells: blockedSet.size,
    expandedNodes: searchResult.expandedNodes,
    uniqueVisitedNodes: searchResult.uniqueVisitedNodes ?? searchResult.expandedNodes,
    relaxationOperations: searchResult.relaxationOperations ?? 0,
    heuristicEvaluations: searchResult.heuristicEvaluations ?? 0,
    executionTimeMs,
    dynamicWindUsed: dynamicMode,
    arrivalStep: searchResult.arrivalStep ?? metrics.steps,
  };
}

export function runEnergyAwareThetaPlaceholder(model) {
  const blockedSet = model.blockedSet ?? new Set();

  return {
    algorithm: "Energy-Aware Theta* (Placeholder)",
    optimizedFor: "Minimum-energy trajectory (future Theta*)",
    path: [],
    objectiveCost: Number.NaN,
    totalDistance: Number.NaN,
    totalEnergyCost: Number.NaN,
    steps: 0,
    windEnergy: Number.NaN,
    gravityEnergy: Number.NaN,
    turningEnergy: Number.NaN,
    blockedCells: blockedSet.size,
    expandedNodes: 0,
    uniqueVisitedNodes: 0,
    relaxationOperations: 0,
    executionTimeMs: 0,
    dynamicWindUsed: false,
    arrivalStep: -1,
    isPlaceholder: true,
  };
}

export function runBellmanFord(model) {
  const rows = model.altitudeGrid.length;
  const cols = model.altitudeGrid[0].length;
  const startTime = performance.now();
  const blockedSet = model.blockedSet ?? new Set();

  const dynamicMode = model.environment.dynamicWindEnabled;

  let searchResult;

  if (dynamicMode) {
    // For dynamic wind we fall back to time-expanded search (DP-like)
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
    searchResult = bellmanFordGrid({
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
    algorithm: dynamicMode ? "Time-Expanded DP (Bellman-Ford fallback)" : "Bellman-Ford (DP)",
    optimizedFor: dynamicMode
      ? "Minimum-energy trajectory (dynamic wind)"
      : "Minimum-energy trajectory",
    path: searchResult.path,
    objectiveCost: searchResult.objectiveCost,
    totalDistance: metrics.distance,
    totalEnergyCost: metrics.energyCost,
    steps: metrics.steps,
    windEnergy: metrics.windEnergy,
    gravityEnergy: metrics.gravityEnergy,
    turningEnergy: metrics.turningEnergy,
    blockedCells: blockedSet.size,
    expandedNodes: searchResult.expandedNodes,
    uniqueVisitedNodes: searchResult.uniqueVisitedNodes ?? searchResult.expandedNodes,
    relaxationOperations: searchResult.relaxationOperations ?? 0,
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
