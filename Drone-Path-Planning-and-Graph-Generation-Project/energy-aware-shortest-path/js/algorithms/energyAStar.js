import { MinPriorityQueue } from "../priorityQueue.js";

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

export function energyAStarGrid({
  rows,
  cols,
  start,
  end,
  blockedSet,
  weightFn,
  heuristicFn,
}) {
  const nodeCount = rows * cols;
  const startIndex = toIndex(start[0], start[1], cols);
  const endIndex = toIndex(end[0], end[1], cols);

  if (isBlockedCell(start[0], start[1], blockedSet) || isBlockedCell(end[0], end[1], blockedSet)) {
    return {
      path: [],
      objectiveCost: Number.POSITIVE_INFINITY,
      expandedNodes: 0,
      uniqueVisitedNodes: 0,
      relaxationOperations: 0,
      heuristicEvaluations: 0,
    };
  }

  const gScores = new Array(nodeCount).fill(Number.POSITIVE_INFINITY);
  const previous = new Array(nodeCount).fill(-1);
  const closed = new Array(nodeCount).fill(false);
  let expandedNodes = 0;
  let relaxationOperations = 0;
  let heuristicEvaluations = 0;

  const queue = new MinPriorityQueue();
  gScores[startIndex] = 0;
  const startHeuristic = heuristicFn(start, 0);
  heuristicEvaluations += 1;
  queue.push(startHeuristic, startIndex);

  while (queue.size > 0) {
    const currentNode = queue.pop();

    if (currentNode === null) {
      break;
    }

    const currentIndex = currentNode.value;

    if (closed[currentIndex]) {
      continue;
    }

    const [currentRow, currentCol] = toCoord(currentIndex, cols);
    const currentHeuristic = heuristicFn([currentRow, currentCol], 0);
    heuristicEvaluations += 1;
    if (currentNode.priority > gScores[currentIndex] + currentHeuristic) {
      continue;
    }

    closed[currentIndex] = true;
    expandedNodes += 1;

    if (currentIndex === endIndex) {
      break;
    }

    const row = currentRow;
    const col = currentCol;

    for (const [nextRow, nextCol] of neighbors(row, col, rows, cols)) {
      if (isBlockedCell(nextRow, nextCol, blockedSet)) {
        continue;
      }

      const nextIndex = toIndex(nextRow, nextCol, cols);
      if (closed[nextIndex]) {
        continue;
      }

      const edgeCost = weightFn([row, col], [nextRow, nextCol]);
      if (!Number.isFinite(edgeCost) || edgeCost < 0) {
        continue;
      }

      relaxationOperations += 1;
      const candidateG = gScores[currentIndex] + edgeCost;

      if (candidateG < gScores[nextIndex]) {
        gScores[nextIndex] = candidateG;
        previous[nextIndex] = currentIndex;
        const heuristic = heuristicFn([nextRow, nextCol], 0);
        heuristicEvaluations += 1;
        const fScore = candidateG + heuristic;
        queue.push(fScore, nextIndex);
      }
    }
  }

  const hasPath = Number.isFinite(gScores[endIndex]);
  const path = hasPath ? reconstructPath(previous, startIndex, endIndex, cols) : [];
  const uniqueVisitedNodes = closed.reduce((count, seen) => count + (seen ? 1 : 0), 0);

  return {
    path,
    objectiveCost: gScores[endIndex],
    expandedNodes,
    uniqueVisitedNodes,
    relaxationOperations,
    heuristicEvaluations,
  };
}

export function energyAStarGridTimeAware({
  rows,
  cols,
  start,
  end,
  blockedSet,
  weightFn,
  heuristicFn,
  maxSteps,
}) {
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
      heuristicEvaluations: 0,
    };
  }

  const horizon = Math.max(1, Math.floor(maxSteps));
  const gScores = Array.from({ length: horizon + 1 }, () =>
    new Array(nodeCount).fill(Number.POSITIVE_INFINITY),
  );
  const previous = new Map();
  const closed = new Set();
  const uniqueVisited = new Set();
  let expandedNodes = 0;
  let relaxationOperations = 0;
  let heuristicEvaluations = 0;

  const queue = new MinPriorityQueue();
  gScores[0][startIndex] = 0;
  const startHeuristic = heuristicFn(start, 0);
  heuristicEvaluations += 1;
  queue.push(startHeuristic, { index: startIndex, step: 0 });

  let bestEndKey = null;

  while (queue.size > 0) {
    const currentNode = queue.pop();

    if (currentNode === null) {
      break;
    }

    const { index: currentIndex, step: currentStep } = currentNode.value;
    const stateKey = timedStateKey(currentIndex, currentStep);

    if (closed.has(stateKey)) {
      continue;
    }

    const knownCost = gScores[currentStep][currentIndex];
    const [currentRow, currentCol] = toCoord(currentIndex, cols);
    const currentHeuristic = heuristicFn([currentRow, currentCol], currentStep);
    heuristicEvaluations += 1;
    if (currentNode.priority > knownCost + currentHeuristic) {
      continue;
    }

    closed.add(stateKey);
    expandedNodes += 1;
    uniqueVisited.add(currentIndex);

    if (currentIndex === endIndex) {
      bestEndKey = stateKey;
      break;
    }

    if (currentStep >= horizon) {
      continue;
    }

    const row = currentRow;
    const col = currentCol;

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
      const candidateG = knownCost + edgeCost;

      if (candidateG < gScores[nextStep][nextIndex]) {
        gScores[nextStep][nextIndex] = candidateG;
        const nextKey = timedStateKey(nextIndex, nextStep);
        previous.set(nextKey, stateKey);
        const heuristic = heuristicFn([nextRow, nextCol], nextStep);
        heuristicEvaluations += 1;
        const fScore = candidateG + heuristic;
        queue.push(fScore, { index: nextIndex, step: nextStep });
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
      heuristicEvaluations,
    };
  }

  const { index: finalIndex, step: arrivalStep } = parseTimedStateKey(bestEndKey);
  const objectiveCost = gScores[arrivalStep][finalIndex];
  const path = reconstructTimedPath(previous, bestEndKey, cols);

  return {
    path,
    objectiveCost,
    expandedNodes,
    arrivalStep,
    uniqueVisitedNodes: uniqueVisited.size,
    relaxationOperations,
    heuristicEvaluations,
  };
}
