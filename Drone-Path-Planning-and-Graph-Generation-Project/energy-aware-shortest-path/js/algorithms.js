import { MinPriorityQueue } from "./priorityQueue.js";
import { computeEnergyComponents } from "./energy/totalEnergy.js";
import {
  energyAStarGrid,
  energyAStarGridTimeAware,
} from "./algorithms/energyAStar.js";
import { energyThetaStarGrid } from "./algorithms/thetaStar.js";
import {
  buildWindTimeline,
  createWindEvolutionSummary,
  getWindChangeSteps,
  getWindStateAtStep as resolveDynamicWindStateAtStep,
  normalizeDynamicEnvironment,
} from "./environment/dynamicWind.js";

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
  turningCoefficient: 0.22,
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

function sameCoord(a, b) {
  if (!Array.isArray(a) || !Array.isArray(b)) {
    return false;
  }

  return a[0] === b[0] && a[1] === b[1];
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
  return resolveDynamicWindStateAtStep(step, environment);
}

function normalizeEnergyEnvironment(environment) {
  const normalized = normalizeDynamicEnvironment(environment);

  return {
    ...normalized,
    distanceCoefficient: normalized.distanceCoefficient ?? 1.0,
    windCoefficient: normalized.windCoefficient ?? 1.0,
    turningCoefficient: normalized.turningCoefficient ?? 0.0,
    mass: normalized.mass ?? normalized.altitudeFactor ?? 1.0,
    gravity: normalized.gravity ?? 9.81,
  };
}

export function computeEnergyEdgeCostWithTurn(from, to, prev, altitudeGrid, environment, step = 0) {
  const windState = getWindStateAtStep(step, environment);
  const energyEnv = normalizeEnergyEnvironment(environment);
  const components = computeEnergyComponents({
    from,
    to,
    prev,
    altitudeGrid,
    environment: energyEnv,
    windState,
    directionVectors: DIRECTION_VECTORS,
  });

  return components.totalEnergy;
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
      turningCount: 0,
      averageHeadingChange: 0,
      smoothnessScore: 0,
    };
  }

  let distance = 0;
  let energyCost = 0;
  let windEnergy = 0;
  let gravityEnergy = 0;
  let turningEnergy = 0;
  let turningCount = 0;
  let headingChangeTotal = 0;
  const energyEnv = normalizeEnergyEnvironment(environment);

  for (let i = 0; i < path.length - 1; i += 1) {
    const from = path[i];
    const to = path[i + 1];
    const prev = i > 0 ? path[i - 1] : null;
    const windState = getWindStateAtStep(i, environment, from, to);
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

    if (prev) {
      const angle = components.turningAngle ?? 0;
      if (angle > 1e-6) {
        turningCount += 1;
        headingChangeTotal += angle;
      }
    }
  }

  const averageHeadingChange = turningCount > 0 ? headingChangeTotal / turningCount : 0;
  const smoothnessScore = 1 - Math.min(1, averageHeadingChange / Math.PI);

  return {
    distance,
    energyCost,
    steps: Math.max(0, path.length - 1),
    windEnergy,
    gravityEnergy,
    turningEnergy,
    turningCount,
    averageHeadingChange,
    smoothnessScore,
  };
}

export function evaluatePathTimeline(path, altitudeGrid, environment) {
  if (!path || path.length < 2) {
    return {
      samples: [],
      energyFluctuation: 0,
      energyVariance: 0,
      energyRange: 0,
      windShiftCount: 0,
    };
  }

  const energyEnv = normalizeEnergyEnvironment(environment);
  const samples = [];
  const energyValues = [];
  let previousSignature = null;
  let windShiftCount = 0;

  for (let i = 0; i < path.length - 1; i += 1) {
    const from = path[i];
    const to = path[i + 1];
    const prev = i > 0 ? path[i - 1] : null;
    const windState = getWindStateAtStep(i, environment, from, to);
    const components = computeEnergyComponents({
      from,
      to,
      prev,
      altitudeGrid,
      environment: energyEnv,
      windState,
      directionVectors: DIRECTION_VECTORS,
    });

    const energyValue = components.totalEnergy;
    const signature = `${windState.direction}|${windState.strength.toFixed(3)}|${windState.hasGust ? 1 : 0}`;

    if (previousSignature !== null && signature !== previousSignature) {
      windShiftCount += 1;
    }

    samples.push({
      step: i,
      from,
      to,
      windDirection: windState.direction,
      windStrength: windState.strength,
      totalEnergy: energyValue,
      windEnergy: components.windEnergy,
      gravityEnergy: components.gravityEnergy,
      turningEnergy: components.turningEnergy,
    });

    energyValues.push(energyValue);
    previousSignature = signature;
  }

  const average = energyValues.reduce((sum, value) => sum + value, 0) / energyValues.length;
  const variance = energyValues.reduce((sum, value) => sum + (value - average) ** 2, 0) / energyValues.length;
  const energyRange = Math.max(...energyValues) - Math.min(...energyValues);
  const energyFluctuation = Math.sqrt(Math.max(0, variance));

  return {
    samples,
    energyFluctuation,
    energyVariance: variance,
    energyRange,
    windShiftCount,
  };
}

function comparePathSimilarity(previousPlan, nextPlan) {
  if (!previousPlan || !nextPlan || previousPlan.length === 0 || nextPlan.length === 0) {
    return 1;
  }

  const limit = Math.min(previousPlan.length, nextPlan.length);
  let matched = 0;

  while (matched < limit) {
    const previousNode = previousPlan[matched];
    const nextNode = nextPlan[matched];

    if (!previousNode || !nextNode) {
      break;
    }

    if (previousNode[0] !== nextNode[0] || previousNode[1] !== nextNode[1]) {
      break;
    }

    matched += 1;
  }

  return matched / Math.max(previousPlan.length, nextPlan.length, 1);
}

function createDynamicEnvironmentSnapshot(environment, step, from = null, to = null) {
  const windState = getWindStateAtStep(step, environment, from, to);
  const normalized = normalizeDynamicEnvironment(environment);

  return {
    ...normalized,
    dynamicWindEnabled: false,
    windDirection: windState.direction,
    windStrength: windState.strength,
    windShifts: [],
    gustRegions: [],
  };
}

function createDynamicResultDefaults() {
  return {
    dynamicWindUsed: false,
    dynamicReplanning: false,
    environmentShiftCount: 0,
    replanCount: 0,
    routeStability: 1,
    energyFluctuation: 0,
    energyVariance: 0,
    energyRange: 0,
    adaptationOverheadMs: 0,
    totalPlanningTimeMs: 0,
    dynamicTimeline: { states: [], shiftCount: 0, gustCount: 0, eventCount: 0 },
    dynamicSummary: { enabled: false, replanCount: 0, environmentShiftCount: 0, routeStability: 1, energyFluctuation: 0 },
  };
}

function finalizeDynamicResult(baseResult, model, metrics, timeline, planningTimes, planningSummary) {
  const dynamicSummary = {
    enabled: true,
    replanCount: planningSummary.replanCount,
    environmentShiftCount: planningSummary.environmentShiftCount,
    routeStability: planningSummary.routeStability,
    energyFluctuation: metrics.energyFluctuation,
    energyVariance: metrics.energyVariance,
    energyRange: metrics.energyRange,
    adaptationOverheadMs: planningSummary.adaptationOverheadMs,
    totalPlanningTimeMs: planningSummary.totalPlanningTimeMs,
    timeline,
    segments: planningTimes.segments,
    averageSegmentPlanningTimeMs: planningTimes.segments.length > 0
      ? planningSummary.totalPlanningTimeMs / planningTimes.segments.length
      : 0,
  };

  return {
    ...baseResult,
    ...planningSummary,
    totalDistance: metrics.distance,
    totalEnergyCost: metrics.energyCost,
    steps: metrics.steps,
    windEnergy: metrics.windEnergy,
    gravityEnergy: metrics.gravityEnergy,
    turningEnergy: metrics.turningEnergy,
    turningCount: metrics.turningCount,
    averageHeadingChange: metrics.averageHeadingChange,
    smoothnessScore: metrics.smoothnessScore,
    blockedCells: model.blockedSet?.size ?? 0,
    dynamicWindUsed: true,
    dynamicReplanning: planningSummary.replanCount > 0,
    energyFluctuation: metrics.energyFluctuation,
    energyVariance: metrics.energyVariance,
    energyRange: metrics.energyRange,
    dynamicTimeline: timeline,
    dynamicSummary,
  };
}

function runAdaptiveDynamicPlanner(model, planner, algorithmName, optimizedForLabel) {
  const rows = model.altitudeGrid.length;
  const cols = model.altitudeGrid[0].length;
  const startTime = performance.now();
  const blockedSet = model.blockedSet ?? new Set();
  const environment = normalizeDynamicEnvironment(model.environment);
  const horizon = computeDynamicHorizon(rows, cols, environment);
  const timeline = buildWindTimeline(environment, horizon);
  const combinedPath = [model.start];
  const segments = [];
  let currentStart = model.start;
  let currentStep = 0;
  let nextPlannedResidual = null;
  let replanCount = 0;
  let environmentShiftCount = 0;
  let routeStabilityTotal = 0;
  let routeStabilitySamples = 0;
  let totalPlanningTimeMs = 0;
  let initialPlanningTimeMs = 0;

  while (currentStep <= horizon && !sameCoord(currentStart, model.end)) {
    const windState = getWindStateAtStep(currentStep, environment, currentStart, model.end);
    const segmentEnvironment = createDynamicEnvironmentSnapshot(environment, currentStep, currentStart, model.end);
    const segmentModel = {
      ...model,
      start: currentStart,
      environment: segmentEnvironment,
      blockedSet,
    };

    const segmentStartTime = performance.now();
    const segmentResult = planner(segmentModel);
    const segmentEndTime = performance.now();
    const segmentPlanningTimeMs = segmentEndTime - segmentStartTime;
    totalPlanningTimeMs += segmentPlanningTimeMs;

    if (segments.length === 0) {
      initialPlanningTimeMs = segmentPlanningTimeMs;
    }

    if (!segmentResult.path || segmentResult.path.length === 0) {
      return {
        algorithm: algorithmName,
        optimizedFor: optimizedForLabel,
        path: [],
        objectiveCost: Number.POSITIVE_INFINITY,
        totalDistance: Number.POSITIVE_INFINITY,
        totalEnergyCost: Number.POSITIVE_INFINITY,
        steps: 0,
        windEnergy: Number.POSITIVE_INFINITY,
        gravityEnergy: Number.POSITIVE_INFINITY,
        turningEnergy: Number.POSITIVE_INFINITY,
        turningCount: 0,
        averageHeadingChange: 0,
        smoothnessScore: 0,
        blockedCells: blockedSet.size,
        expandedNodes: 0,
        uniqueVisitedNodes: 0,
        relaxationOperations: 0,
        heuristicEvaluations: 0,
        executionTimeMs: segmentPlanningTimeMs,
        runtimeMs: segmentPlanningTimeMs,
        avgBenchmarkTime: null,
        arrivalStep: -1,
        ...createDynamicResultDefaults(),
      };
    }

    if (segmentResult.path.length === 1 && !sameCoord(segmentResult.path[0], model.end)) {
      return {
        algorithm: algorithmName,
        optimizedFor: optimizedForLabel,
        path: [],
        objectiveCost: Number.POSITIVE_INFINITY,
        totalDistance: Number.POSITIVE_INFINITY,
        totalEnergyCost: Number.POSITIVE_INFINITY,
        steps: 0,
        windEnergy: Number.POSITIVE_INFINITY,
        gravityEnergy: Number.POSITIVE_INFINITY,
        turningEnergy: Number.POSITIVE_INFINITY,
        turningCount: 0,
        averageHeadingChange: 0,
        smoothnessScore: 0,
        blockedCells: blockedSet.size,
        expandedNodes: segmentResult.expandedNodes ?? 0,
        uniqueVisitedNodes: segmentResult.uniqueVisitedNodes ?? 0,
        relaxationOperations: segmentResult.relaxationOperations ?? 0,
        heuristicEvaluations: segmentResult.heuristicEvaluations ?? 0,
        executionTimeMs: segmentPlanningTimeMs,
        runtimeMs: segmentPlanningTimeMs,
        avgBenchmarkTime: null,
        arrivalStep: -1,
        ...createDynamicResultDefaults(),
      };
    }

    if (nextPlannedResidual) {
      routeStabilityTotal += comparePathSimilarity(nextPlannedResidual, segmentResult.path);
      routeStabilitySamples += 1;
      nextPlannedResidual = null;
    }

    const nextActiveStep = getWindChangeSteps(environment).find((step) => step > currentStep) ?? Number.POSITIVE_INFINITY;
    const executedSteps = Math.min(segmentResult.path.length - 1, Math.max(0, nextActiveStep - currentStep));
    const executedPath = segmentResult.path.slice(0, executedSteps + 1);

    if (combinedPath.length === 1) {
      combinedPath.push(...executedPath.slice(1));
    } else {
      combinedPath.push(...executedPath.slice(1));
    }

    segments.push({
      step: currentStep,
      executedSteps,
      plannedSteps: Math.max(0, segmentResult.path.length - 1),
      windDirection: windState.direction,
      windStrength: windState.strength,
      planningTimeMs: segmentPlanningTimeMs,
      path: segmentResult.path,
      executedPath,
      expandedNodes: segmentResult.expandedNodes ?? 0,
      uniqueVisitedNodes: segmentResult.uniqueVisitedNodes ?? 0,
      relaxationOperations: segmentResult.relaxationOperations ?? 0,
      heuristicEvaluations: segmentResult.heuristicEvaluations ?? 0,
    });

    currentStart = executedPath[executedPath.length - 1];
    currentStep += executedSteps;

    if (sameCoord(currentStart, model.end) || executedSteps >= segmentResult.path.length - 1) {
      break;
    }

    nextPlannedResidual = segmentResult.path.slice(Math.max(0, executedSteps));
    replanCount += 1;
    environmentShiftCount += 1;

    if (currentStep >= horizon) {
      break;
    }
  }

  const executionTimeMs = performance.now() - startTime;
  const metrics = evaluatePathMetrics(combinedPath, model.altitudeGrid, environment);
  const timelineMetrics = evaluatePathTimeline(combinedPath, model.altitudeGrid, environment);
  const planningSummary = {
    replanCount,
    environmentShiftCount,
    routeStability: routeStabilitySamples > 0 ? routeStabilityTotal / routeStabilitySamples : 1,
    adaptationOverheadMs: Math.max(0, totalPlanningTimeMs - initialPlanningTimeMs),
    totalPlanningTimeMs,
  };

  return finalizeDynamicResult(
    {
      algorithm: algorithmName,
      optimizedFor: optimizedForLabel,
      path: combinedPath,
      objectiveCost: metrics.energyCost,
      totalDistance: metrics.distance,
      totalEnergyCost: metrics.energyCost,
      steps: metrics.steps,
      windEnergy: metrics.windEnergy,
      gravityEnergy: metrics.gravityEnergy,
      turningEnergy: metrics.turningEnergy,
      turningCount: metrics.turningCount,
      averageHeadingChange: metrics.averageHeadingChange,
      smoothnessScore: metrics.smoothnessScore,
      blockedCells: blockedSet.size,
      expandedNodes: segments.reduce((sum, segment) => sum + (segmentResultCount(segment) || 0), 0),
      uniqueVisitedNodes: segments.reduce((sum, segment) => sum + (segmentResultUnique(segment) || 0), 0),
      relaxationOperations: segments.reduce((sum, segment) => sum + (segment.relaxationOperations ?? 0), 0),
      heuristicEvaluations: segments.reduce((sum, segment) => sum + (segment.heuristicEvaluations ?? 0), 0),
      executionTimeMs,
      runtimeMs: executionTimeMs,
      avgBenchmarkTime: null,
      arrivalStep: metrics.steps,
    },
    model,
    {
      ...metrics,
      energyFluctuation: timelineMetrics.energyFluctuation,
      energyVariance: timelineMetrics.energyVariance,
      energyRange: timelineMetrics.energyRange,
    },
    timeline,
    { segments },
    planningSummary,
  );
}

function segmentResultCount(segment) {
  return Number.isFinite(segment?.expandedNodes) ? segment.expandedNodes : 0;
}

function segmentResultUnique(segment) {
  return Number.isFinite(segment?.uniqueVisitedNodes) ? segment.uniqueVisitedNodes : 0;
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
    turningCount: metrics.turningCount,
    averageHeadingChange: metrics.averageHeadingChange,
    smoothnessScore: metrics.smoothnessScore,
    blockedCells: blockedSet.size,
    expandedNodes: searchResult.expandedNodes,
    uniqueVisitedNodes: searchResult.uniqueVisitedNodes ?? searchResult.expandedNodes,
    relaxationOperations: searchResult.relaxationOperations ?? 0,
    executionTimeMs,
    dynamicWindUsed: false,
    dynamicReplanning: false,
    environmentShiftCount: 0,
    replanCount: 0,
    routeStability: 1,
    energyFluctuation: 0,
    energyVariance: 0,
    energyRange: 0,
    adaptationOverheadMs: 0,
    totalPlanningTimeMs: 0,
    dynamicTimeline: { states: [], shiftCount: 0, gustCount: 0, eventCount: 0 },
    dynamicSummary: createDynamicResultDefaults().dynamicSummary,
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
    turningCount: metrics.turningCount,
    averageHeadingChange: metrics.averageHeadingChange,
    smoothnessScore: metrics.smoothnessScore,
    blockedCells: blockedSet.size,
    expandedNodes: searchResult.expandedNodes,
    uniqueVisitedNodes: searchResult.uniqueVisitedNodes ?? searchResult.expandedNodes,
    relaxationOperations: searchResult.relaxationOperations ?? 0,
    heuristicEvaluations: searchResult.heuristicEvaluations ?? 0,
    executionTimeMs,
    dynamicWindUsed: dynamicMode,
    arrivalStep: searchResult.arrivalStep ?? metrics.steps,
    dynamicReplanning: false,
    environmentShiftCount: dynamicMode ? buildWindTimeline(model.environment, metrics.steps).shiftCount : 0,
    replanCount: 0,
    routeStability: 1,
    energyFluctuation: 0,
    energyVariance: 0,
    energyRange: 0,
    adaptationOverheadMs: 0,
    totalPlanningTimeMs: executionTimeMs,
    dynamicTimeline: buildWindTimeline(model.environment, metrics.steps),
    dynamicSummary: {
      enabled: dynamicMode,
      replanCount: 0,
      environmentShiftCount: dynamicMode ? buildWindTimeline(model.environment, metrics.steps).shiftCount : 0,
      routeStability: 1,
      energyFluctuation: 0,
      energyVariance: 0,
      energyRange: 0,
      adaptationOverheadMs: 0,
      totalPlanningTimeMs: executionTimeMs,
    },
  };
}

export function runEnergyAwareThetaStar(model) {
  const rows = model.altitudeGrid.length;
  const cols = model.altitudeGrid[0].length;
  const startTime = performance.now();
  const blockedSet = model.blockedSet ?? new Set();

  const heuristicBase = model.environment.averageEnergyFactor ?? model.environment.distanceCoefficient ?? 1.0;
  const energyCoefficient = Math.max(0, heuristicBase);
  const heuristicFn = (coord) => energyCoefficient * geometricDistance(coord, model.end);

  const dynamicMode = model.environment.dynamicWindEnabled;
  const optimizedFor = dynamicMode
    ? "Minimum-energy trajectory (static wind approximation)"
    : "Minimum-energy trajectory (Theta*)";

  const searchResult = energyThetaStarGrid({
    rows,
    cols,
    start: model.start,
    end: model.end,
    blockedSet,
    heuristicFn,
    weightFn: (from, to, prev) =>
      computeEnergyEdgeCostWithTurn(
        from,
        to,
        prev,
        model.altitudeGrid,
        model.environment,
        0,
      ),
    debug: model.debugTheta ?? false,
  });

  const executionTimeMs = performance.now() - startTime;
  const metricPath = searchResult.path;
  const metrics = evaluatePathMetrics(metricPath, model.altitudeGrid, model.environment);
  const theory = {
    recurrenceRelation: "No standard divide-and-conquer recurrence. Theta* extends A* using line-of-sight relaxation.",
    recurrenceApplicability: "Theta* is analyzed similarly to A* with additional LOS checks.",
    bestCaseComplexity: "O((V + E) log V)",
    averageCaseComplexity: "O((V + E) log V)",
    worstCaseComplexity: "O((V + E) log V)",
  };

  return {
    algorithm: "Energy-Aware Theta*",
    algorithmName: "Energy-Aware Theta*",
    optimizedFor,
    optimizationGoal: optimizedFor,
    path: searchResult.expandedPath,
    segmentPath: searchResult.path,
    debug: searchResult.debug ?? null,
    objectiveCost: searchResult.objectiveCost,
    totalEnergy: metrics.energyCost,
    totalDistance: metrics.distance,
    pathDistance: metrics.distance,
    totalEnergyCost: metrics.energyCost,
    steps: Math.max(0, (searchResult.expandedPath?.length ?? 0) - 1),
    windEnergy: metrics.windEnergy,
    gravityEnergy: metrics.gravityEnergy,
    turningEnergy: metrics.turningEnergy,
    turningCount: metrics.turningCount,
    averageHeadingChange: metrics.averageHeadingChange,
    smoothnessScore: metrics.smoothnessScore,
    nodeOperations: searchResult.relaxationOperations ?? 0,
    blockedCells: blockedSet.size,
    expandedNodes: searchResult.expandedNodes,
    uniqueVisitedNodes: searchResult.uniqueVisitedNodes ?? searchResult.expandedNodes,
    relaxationOperations: searchResult.relaxationOperations ?? 0,
    heuristicEvaluations: searchResult.heuristicEvaluations ?? 0,
    executionTimeMs,
    runtimeMs: executionTimeMs,
    avgBenchmarkTime: model.averageBenchmarkTime ?? null,
    dynamicWindUsed: dynamicMode,
    arrivalStep: metrics.steps,
    complexity: {
      bestCase: theory.bestCaseComplexity,
      averageCase: theory.averageCaseComplexity,
      worstCase: theory.worstCaseComplexity,
    },
    theory,
    recurrenceRelation: theory.recurrenceRelation,
    recurrenceApplicability: theory.recurrenceApplicability,
    bestCaseComplexity: theory.bestCaseComplexity,
    averageCaseComplexity: theory.averageCaseComplexity,
    worstCaseComplexity: theory.worstCaseComplexity,
    dynamicReplanning: false,
    environmentShiftCount: dynamicMode ? buildWindTimeline(model.environment, metrics.steps).shiftCount : 0,
    replanCount: 0,
    routeStability: 1,
    energyFluctuation: 0,
    energyVariance: 0,
    energyRange: 0,
    adaptationOverheadMs: 0,
    totalPlanningTimeMs: executionTimeMs,
    dynamicTimeline: buildWindTimeline(model.environment, metrics.steps),
    dynamicSummary: {
      enabled: dynamicMode,
      replanCount: 0,
      environmentShiftCount: dynamicMode ? buildWindTimeline(model.environment, metrics.steps).shiftCount : 0,
      routeStability: 1,
      energyFluctuation: 0,
      energyVariance: 0,
      energyRange: 0,
      adaptationOverheadMs: 0,
      totalPlanningTimeMs: executionTimeMs,
    },
  };
}

export function runAdaptiveEnergyAwareAStar(model) {
  return runAdaptiveDynamicPlanner(
    model,
    runEnergyAwareAStar,
    "Energy-Aware A*",
    "Minimum-energy trajectory (adaptive dynamic wind)",
  );
}

export function runAdaptiveEnergyAwareThetaStar(model) {
  return runAdaptiveDynamicPlanner(
    model,
    runEnergyAwareThetaStar,
    "Energy-Aware Theta*",
    "Minimum-energy trajectory (adaptive dynamic wind)",
  );
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
    turningCount: metrics.turningCount,
    averageHeadingChange: metrics.averageHeadingChange,
    smoothnessScore: metrics.smoothnessScore,
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
