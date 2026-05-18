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

function bresenhamLine(from, to) {
  const points = [];
  let x0 = from[0];
  let y0 = from[1];
  const x1 = to[0];
  const y1 = to[1];
  const dx = Math.abs(x1 - x0);
  const dy = Math.abs(y1 - y0);
  const sx = x0 < x1 ? 1 : -1;
  const sy = y0 < y1 ? 1 : -1;
  let err = dx - dy;

  while (true) {
    points.push([x0, y0]);

    if (x0 === x1 && y0 === y1) {
      break;
    }

    const e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }

  return points;
}

export function lineOfSight(from, to, rows, cols, blockedSet) {
  const points = bresenhamLine(from, to);

  for (const [row, col] of points) {
    if (!inBounds(row, col, rows, cols)) {
      return false;
    }

    if (isBlockedCell(row, col, blockedSet)) {
      return false;
    }
  }

  return true;
}

function reconstructWaypoints(parent, startIndex, endIndex, cols) {
  const path = [];
  let current = endIndex;

  while (current !== -1) {
    path.push(toCoord(current, cols));

    if (current === startIndex) {
      break;
    }

    const next = parent[current];
    if (next === current) {
      break;
    }

    current = next;
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

function densifyPath(path) {
  if (!path || path.length === 0) {
    return [];
  }

  const output = [path[0]];

  for (let i = 0; i < path.length - 1; i += 1) {
    const segment = bresenhamLine(path[i], path[i + 1]);
    for (let j = 1; j < segment.length; j += 1) {
      output.push(segment[j]);
    }
  }

  return output;
}

export function energyThetaStarGrid({
  rows,
  cols,
  start,
  end,
  blockedSet,
  weightFn,
  heuristicFn,
  debug = false,
}) {
  const nodeCount = rows * cols;
  const startIndex = toIndex(start[0], start[1], cols);
  const endIndex = toIndex(end[0], end[1], cols);

  if (isBlockedCell(start[0], start[1], blockedSet) || isBlockedCell(end[0], end[1], blockedSet)) {
    return {
      path: [],
      expandedPath: [],
      objectiveCost: Number.POSITIVE_INFINITY,
      expandedNodes: 0,
      uniqueVisitedNodes: 0,
      relaxationOperations: 0,
      heuristicEvaluations: 0,
    };
  }

  const gScores = new Array(nodeCount).fill(Number.POSITIVE_INFINITY);
  const parent = new Array(nodeCount).fill(-1);
  const closed = new Array(nodeCount).fill(false);
  let expandedNodes = 0;
  let relaxationOperations = 0;
  let heuristicEvaluations = 0;
  let losChecks = 0;
  let losSuccess = 0;

  const queue = new MinPriorityQueue();
  gScores[startIndex] = 0;
  parent[startIndex] = startIndex;
  const startHeuristic = heuristicFn(start);
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
    const currentHeuristic = heuristicFn([currentRow, currentCol]);
    heuristicEvaluations += 1;
    if (currentNode.priority > gScores[currentIndex] + currentHeuristic) {
      continue;
    }

    closed[currentIndex] = true;
    expandedNodes += 1;

    if (currentIndex === endIndex) {
      break;
    }

    for (const [nextRow, nextCol] of neighbors(currentRow, currentCol, rows, cols)) {
      if (isBlockedCell(nextRow, nextCol, blockedSet)) {
        continue;
      }

      const nextIndex = toIndex(nextRow, nextCol, cols);
      if (closed[nextIndex]) {
        continue;
      }

      const parentIndex = parent[currentIndex];
      const parentCoord = parentIndex >= 0 ? toCoord(parentIndex, cols) : null;
      const hasLos = parentCoord && lineOfSight(parentCoord, [nextRow, nextCol], rows, cols, blockedSet);
      if (parentCoord) {
        losChecks += 1;
        if (hasLos) {
          losSuccess += 1;
        }
      }

      let candidateG = Number.POSITIVE_INFINITY;
      let candidateParent = currentIndex;

      if (hasLos) {
        const grandParentIndex = parentIndex >= 0 ? parent[parentIndex] : -1;
        const grandParentCoord =
          grandParentIndex >= 0 && grandParentIndex !== parentIndex
            ? toCoord(grandParentIndex, cols)
            : null;
        const edgeCost = weightFn(parentCoord, [nextRow, nextCol], grandParentCoord);
        if (Number.isFinite(edgeCost) && edgeCost >= 0) {
          relaxationOperations += 1;
          candidateG = gScores[parentIndex] + edgeCost;
          candidateParent = parentIndex;
        }
      }

      if (!Number.isFinite(candidateG)) {
        const prevIndex = parent[currentIndex] >= 0 && parent[currentIndex] !== currentIndex
          ? parent[currentIndex]
          : -1;
        const prevCoord = prevIndex >= 0 ? toCoord(prevIndex, cols) : null;
        const edgeCost = weightFn([currentRow, currentCol], [nextRow, nextCol], prevCoord);
        if (!Number.isFinite(edgeCost) || edgeCost < 0) {
          continue;
        }

        relaxationOperations += 1;
        candidateG = gScores[currentIndex] + edgeCost;
        candidateParent = currentIndex;
      }

      if (candidateG < gScores[nextIndex]) {
        gScores[nextIndex] = candidateG;
        parent[nextIndex] = candidateParent;
        const heuristic = heuristicFn([nextRow, nextCol]);
        heuristicEvaluations += 1;
        queue.push(candidateG + heuristic, nextIndex);
      }
    }
  }

  const hasPath = Number.isFinite(gScores[endIndex]);
  const path = hasPath ? reconstructWaypoints(parent, startIndex, endIndex, cols) : [];
  const expandedPath = hasPath ? densifyPath(path) : [];
  const uniqueVisitedNodes = closed.reduce((count, seen) => count + (seen ? 1 : 0), 0);

  return {
    path,
    expandedPath,
    objectiveCost: gScores[endIndex],
    expandedNodes,
    uniqueVisitedNodes,
    relaxationOperations,
    heuristicEvaluations,
    debug: debug
      ? {
        losChecks,
        losSuccess,
        openSetSize: queue.size,
        hasPath,
      }
      : null,
  };
}
