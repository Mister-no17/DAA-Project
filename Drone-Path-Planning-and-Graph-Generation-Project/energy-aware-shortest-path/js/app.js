import {
  DEFAULT_ENVIRONMENT,
  DIRECTION_VECTORS,
  coordinateKey,
  createRandomAltitudeGrid,
  dijkstraGrid,
  geometricDistance,
  formatCoord,
  runEnergyAwareAStar,
  runStandardDijkstra,
} from "./algorithms.js";
import { PRESET_SCENARIOS, cloneGrid, getScenarioById } from "./scenarios.js";

const elements = {
  startTourBtn: document.getElementById("startTourBtn"),
  scenarioSelect: document.getElementById("scenarioSelect"),
  loadScenarioBtn: document.getElementById("loadScenarioBtn"),
  scenarioDescription: document.getElementById("scenarioDescription"),
  windDirection: document.getElementById("windDirection"),
  windStrength: document.getElementById("windStrength"),
  windStrengthValue: document.getElementById("windStrengthValue"),
  altitudeFactor: document.getElementById("altitudeFactor"),
  altitudeFactorValue: document.getElementById("altitudeFactorValue"),
  dynamicWindEnabled: document.getElementById("dynamicWindEnabled"),
  windShiftControls: document.getElementById("windShiftControls"),
  windShiftStep: document.getElementById("windShiftStep"),
  windShiftStepValue: document.getElementById("windShiftStepValue"),
  windDirectionAfterShift: document.getElementById("windDirectionAfterShift"),
  windStrengthAfterShift: document.getElementById("windStrengthAfterShift"),
  windStrengthAfterShiftValue: document.getElementById("windStrengthAfterShiftValue"),
  showWindFlow: document.getElementById("showWindFlow"),
  obstaclePaintMode: document.getElementById("obstaclePaintMode"),
  obstacleDensity: document.getElementById("obstacleDensity"),
  obstacleDensityValue: document.getElementById("obstacleDensityValue"),
  enablePathGlow: document.getElementById("enablePathGlow"),
  randomObstacleBtn: document.getElementById("randomObstacleBtn"),
  clearObstacleBtn: document.getElementById("clearObstacleBtn"),
  randomizeBtn: document.getElementById("randomizeBtn"),
  resetSelectionBtn: document.getElementById("resetSelectionBtn"),
  runBtn: document.getElementById("runBtn"),
  generateReportBtn: document.getElementById("generateReportBtn"),
  grid: document.getElementById("grid"),
  windFlow: document.getElementById("windFlow"),
  windCompass: document.getElementById("windCompass"),
  windCompassLabel: document.getElementById("windCompassLabel"),
  statsCards: document.getElementById("statsCards"),
  comparisonTable: document.querySelector("#comparisonTable tbody"),
  pathSummary: document.getElementById("pathSummary"),
  resultExplanation: document.getElementById("resultExplanation"),
  statusLine: document.getElementById("statusLine"),
  enablePathGlow: document.getElementById("enablePathGlow"),
  tourOverlay: document.getElementById("tourOverlay"),
  tourStepCounter: document.getElementById("tourStepCounter"),
  tourTitle: document.getElementById("tourTitle"),
  tourBody: document.getElementById("tourBody"),
  tourBackBtn: document.getElementById("tourBackBtn"),
  tourSkipBtn: document.getElementById("tourSkipBtn"),
  tourNextBtn: document.getElementById("tourNextBtn"),
};

const DEFAULT_OBSTACLE_DENSITY = 0.18;
const TOUR_DISMISSED_STORAGE_KEY = "energy-aware-tour-dismissed";
const REPORT_SERVER_URL = "http://localhost:8001/report";
const BENCHMARK_ITERATIONS = 100;
let windFlowEnabled = true;
const TOUR_STEPS = [
  {
    selector: ".hero",
    title: "Project Overview",
    body: "This header summarizes the goal: compare distance-minimizing routing against minimum-energy trajectories under wind and altitude effects.",
  },
  {
    selector: "#startTourBtn",
    title: "Guided Tour Button",
    body: "Use this button anytime to restart this walkthrough. You can skip from any step.",
  },
  {
    selector: "#scenarioSelect",
    title: "Preset Scenario",
    body: "Choose a predefined map with start/end points, environment settings, and obstacle layout.",
  },
  {
    selector: "#loadScenarioBtn",
    title: "Load Scenario",
    body: "Applies the selected scenario to the grid and resets previous results.",
  },
  {
    selector: "#windDirection",
    title: "Wind Direction",
    body: "Sets the wind vector direction that influences movement cost in the energy-aware solver.",
  },
  {
    selector: "#windStrength",
    title: "Wind Strength",
    body: "Controls how strongly wind affects energy cost. Higher values penalize moving against wind more.",
  },
  {
    selector: "#altitudeFactor",
    title: "Drone Mass Scale",
    body: "Scales how expensive uphill climbs are. Higher values encourage flatter, energy-efficient routes.",
  },
  {
    selector: "#dynamicWindEnabled",
    title: "Sudden Wind Shift Toggle",
    body: "When enabled, wind can change mid-route after a configurable step.",
  },
  {
    selector: "#windShiftControls",
    title: "Wind Shift Controls",
    body: "Set when the shift happens and what the post-shift wind direction and strength become.",
  },
  {
    selector: "#obstaclePaintMode",
    title: "Obstacle Paint Mode",
    body: "Enable this to edit obstacles manually. Use Shift + Click on grid cells to add or remove no-fly nodes.",
  },
  {
    selector: "#obstacleDensity",
    title: "Random Obstacle Density",
    body: "Defines the probability used when generating random obstacles.",
  },
  {
    selector: "#randomObstacleBtn",
    title: "Generate Obstacles",
    body: "Builds a random obstacle map using the selected density while trying to keep a feasible route.",
  },
  {
    selector: "#clearObstacleBtn",
    title: "Clear Obstacles",
    body: "Removes all blocked cells from the current map.",
  },
  {
    selector: "#randomizeBtn",
    title: "Randomize Altitude",
    body: "Generates a new altitude landscape and clears obstacles/results for a fresh run.",
  },
  {
    selector: "#resetSelectionBtn",
    title: "Reset Start/End",
    body: "Clears selected start and end nodes so you can choose new endpoints.",
  },
  {
    selector: "#runBtn",
    title: "Run All Algorithms",
    body: "Executes standard Dijkstra and Energy-Aware A* for the current grid and settings.",
  },
  {
    selector: "#statusLine",
    title: "Status Line",
    body: "Shows immediate guidance such as selection state, validation messages, and run outcomes.",
  },
  {
    selector: "#grid",
    title: "Grid Visualization",
    body: "Click cells to set start and end nodes. Cell color indicates altitude and overlays show computed paths.",
  },
  {
    selector: ".legend",
    title: "Legend",
    body: "Explains start/end markers, each algorithm path color, overlap cells, and blocked cells.",
  },
  {
    selector: "#statsCards",
    title: "Metric Cards",
    body: "Presents compact metrics for each algorithm: distance, energy, penalties, execution time, and node expansions.",
  },
  {
    selector: "#comparisonTable",
    title: "Comparison Table",
    body: "Shows metric-by-metric values side by side so differences are easy to inspect.",
  },
  {
    selector: "#pathSummary",
    title: "Path Summary",
    body: "Lists start/end, wind mode, whether paths differ, and condensed route strings for both algorithms.",
  },
  {
    selector: "#resultExplanation",
    title: "Case Explanation",
    body: "After each run, this section explains how the three algorithms compare in this exact scenario.",
  },
];

const THEORY_PROFILES = Object.freeze({
  standardDijkstra: {
    recurrence: "No standard recurrence relation (greedy graph algorithm).",
    recurrenceNote:
      "Dijkstra is usually analyzed with graph operations and a priority queue, not with divide-and-conquer recurrence equations.",
    bestCase: "O((V + E) log V)",
    averageCase: "O((V + E) log V)",
    worstCase: "O((V + E) log V)",
  },
  energyAStarStatic: {
    recurrence: "No standard recurrence relation (greedy graph algorithm).",
    recurrenceNote:
      "Energy-aware A* uses a Dijkstra-style priority queue here, so complexity is analyzed like Dijkstra for now.",
    bestCase: "O((V + E) log V)",
    averageCase: "O((V + E) log V)",
    worstCase: "O((V + E) log V)",
  },
  energyAStarDynamic: {
    recurrence: "No standard recurrence relation (time-expanded shortest path solved greedily).",
    recurrenceNote:
      "With dynamic wind, states include time-step. Complexity is analyzed on the expanded graph instead of a classic recurrence.",
    bestCase: "O((V_t + E_t) log V_t)",
    averageCase: "O((V_t + E_t) log V_t)",
    worstCase: "O((V_t + E_t) log V_t)",
  },
});

function createEnvironment(overrides = {}) {
  const merged = {
    ...DEFAULT_ENVIRONMENT,
    ...overrides,
  };

  if (!Number.isFinite(merged.mass)) {
    merged.mass = merged.altitudeFactor ?? DEFAULT_ENVIRONMENT.mass;
  }

  return merged;
}

const state = {
  altitudeGrid: [],
  start: null,
  end: null,
  environment: createEnvironment(),
  blockedCells: new Set(),
  paintObstacles: false,
  pathGlowEnabled: true,
  results: null,
  activeScenarioId: null,
  tourActive: false,
  tourStepIndex: 0,
  tourFocusedElement: null,
};

function sameCoord(a, b) {
  if (!a || !b) {
    return false;
  }

  return a[0] === b[0] && a[1] === b[1];
}

function formatNumber(value, digits = 2) {
  if (!Number.isFinite(value)) {
    return "N/A";
  }

  return value.toFixed(digits);
}

function formatSignedNumber(value, digits = 2) {
  if (!Number.isFinite(value)) {
    return "N/A";
  }

  const sign = value > 0 ? "+" : "";
  return `${sign}${value.toFixed(digits)}`;
}

function percentDelta(referenceValue, candidateValue) {
  if (!Number.isFinite(referenceValue) || !Number.isFinite(candidateValue)) {
    return null;
  }

  if (Math.abs(referenceValue) < 0.000001) {
    return null;
  }

  return ((candidateValue - referenceValue) / referenceValue) * 100;
}

function computeGridStats(grid, blockedSet) {
  if (!grid || grid.length === 0) {
    return { vertices: 0, edges: 0, rows: 0, cols: 0 };
  }

  const rows = grid.length;
  const cols = grid[0].length;
  let vertices = 0;
  let edges = 0;

  for (let row = 0; row < rows; row += 1) {
    for (let col = 0; col < cols; col += 1) {
      const key = `${row},${col}`;
      if (blockedSet.has(key)) {
        continue;
      }

      vertices += 1;

      if (row + 1 < rows && !blockedSet.has(`${row + 1},${col}`)) {
        edges += 1;
      }

      if (col + 1 < cols && !blockedSet.has(`${row},${col + 1}`)) {
        edges += 1;
      }
    }
  }

  return { vertices, edges, rows, cols };
}

function estimateOperations(algorithmName, vertices, edges) {
  if (vertices <= 1 || edges <= 0) {
    return 0;
  }

  return (vertices + edges) * Math.log2(vertices);
}

function formatOps(value) {
  if (!Number.isFinite(value)) {
    return "N/A";
  }

  if (value < 1000) {
    return Math.round(value).toString();
  }

  return value.toLocaleString("en-US", { maximumFractionDigits: 0 });
}

function formatPercent(value, digits = 1) {
  if (!Number.isFinite(value)) {
    return "N/A";
  }

  return `${(value * 100).toFixed(digits)}%`;
}

function formatRatio(value, digits = 2) {
  if (!Number.isFinite(value)) {
    return "N/A";
  }

  return value.toFixed(digits);
}

function buildRuntimeMetrics(result, gridStats, actualExecutionTime) {
  const vertices = gridStats.vertices;
  const edges = gridStats.edges;
  const estimatedWorkload = Math.round((vertices + edges) * Math.log2(vertices || 1));

  return {
    theoreticalComplexity: "O((V + E) log V)",
    runtimeFormula: "(V + E) log2(V)",
    estimatedWorkload,
    actualExecutionTime,
    averageExecutionTime: result.averageExecutionTime ?? actualExecutionTime,
  };
}

function computeDynamicHorizon(rows, cols, environment) {
  return Math.max(
    rows + cols,
    Math.floor(rows * cols * (environment.maxStepMultiplier ?? 2.5)),
  );
}

function benchmarkStandard(model) {
  const rows = model.altitudeGrid.length;
  const cols = model.altitudeGrid[0].length;
  const blockedSet = model.blockedSet ?? new Set();
  const startTime = performance.now();

  for (let i = 0; i < BENCHMARK_ITERATIONS; i += 1) {
    dijkstraGrid({
      rows,
      cols,
      start: model.start,
      end: model.end,
      blockedSet,
      weightFn: geometricDistance,
    });
  }

  const endTime = performance.now();
  return (endTime - startTime) / BENCHMARK_ITERATIONS;
}

function benchmarkEnergyAwareAStar(model) {
  const startTime = performance.now();

  for (let i = 0; i < BENCHMARK_ITERATIONS; i += 1) {
    runEnergyAwareAStar(model);
  }

  const endTime = performance.now();
  return (endTime - startTime) / BENCHMARK_ITERATIONS;
}

function windDirectionAngle(direction) {
  const map = {
    N: 0,
    NE: 45,
    E: 90,
    SE: 135,
    S: 180,
    SW: 225,
    W: 270,
    NW: 315,
  };

  return map[direction] ?? 90;
}

function windFlowDuration(strength) {
  if (!Number.isFinite(strength)) {
    return 8;
  }

  return Math.max(4, 10 - strength * 1.2);
}

function updateWindIndicators() {
  if (!elements.windCompass) {
    return;
  }

  const direction = state.environment.windDirection ?? "E";
  const angle = windDirectionAngle(direction);
  const flowSeconds = windFlowDuration(state.environment.windStrength);
  const dynamicEnabled = state.environment.dynamicWindEnabled;
  const afterDirection = state.environment.windDirectionAfterShift ?? direction;
  const labelText = dynamicEnabled ? `Wind: ${direction} -> ${afterDirection}` : `Wind: ${direction}`;

  elements.windCompass.style.setProperty("--wind-angle", `${angle}deg`);
  elements.windCompassLabel.textContent = labelText;
  elements.windCompass.title = dynamicEnabled
    ? `Wind shifts at step ${state.environment.windShiftStep}: ${direction} to ${afterDirection}`
    : `Wind direction: ${direction}`;

  if (elements.windFlow) {
    elements.windFlow.style.setProperty("--wind-angle", `${angle}deg`);
    elements.windFlow.style.setProperty("--wind-flow-duration", `${flowSeconds}s`);
  }
}

function updateWindFlowVisibility() {
  if (!elements.windFlow) {
    return;
  }

  elements.windFlow.classList.toggle("wind-flow-hidden", !windFlowEnabled);
}

function metricBadge(label) {
  return `<span class="metric-badge">${label}</span>`;
}

function getScenarioLabel() {
  const selected = elements.scenarioSelect?.value;
  const scenario = selected ? getScenarioById(selected) : null;
  return scenario?.name ?? "Custom";
}

function buildReportPayload() {
  if (!state.results || !state.start || !state.end) {
    return null;
  }

  const gridStats = computeGridStats(state.altitudeGrid, state.blockedCells);
  const algorithms = [state.results.standard, state.results.energy];

  return {
    scenarioName: getScenarioLabel(),
    timestampIso: new Date().toISOString(),
    grid: {
      rows: gridStats.rows,
      cols: gridStats.cols,
      vertices: gridStats.vertices,
      edges: gridStats.edges,
    },
    environment: { ...state.environment },
    obstacles: Array.from(state.blockedCells),
    start: state.start,
    end: state.end,
    altitudeGrid: state.altitudeGrid,
    algorithms: algorithms.map((result) => ({
      name: result.algorithm,
      optimizedFor: result.optimizedFor,
      objectiveCost: result.objectiveCost,
      totalDistance: result.totalDistance,
      totalEnergyCost: result.totalEnergyCost,
      windEnergy: result.windEnergy,
      gravityEnergy: result.gravityEnergy,
      turningEnergy: result.turningEnergy,
      executionTimeMs: result.executionTimeMs,
      executionTime: result.executionTime ?? result.actualExecutionTime ?? result.executionTimeMs,
      averageExecutionTime: result.averageExecutionTime ?? null,
      expandedNodes: result.expandedNodes,
      uniqueVisitedNodes: result.uniqueVisitedNodes ?? result.expandedNodes,
      relaxationOperations: result.relaxationOperations ?? 0,
      heuristicEvaluations: result.heuristicEvaluations ?? 0,
      steps: result.steps,
      dynamicWindUsed: result.dynamicWindUsed,
      estimatedOperations: estimateOperations(result.algorithm, gridStats.vertices, gridStats.edges),
    })),
    paths: {
      standard: state.results.standard.path,
      energyAStar: state.results.energy.path,
    },
  };
}

function setStatus(text) {
  elements.statusLine.textContent = text;
}

function getTheoryProfile(result) {
  if (result.algorithm === "Standard Dijkstra") {
    return THEORY_PROFILES.standardDijkstra;
  }

  if (result.algorithm === "Energy-Aware A*") {
    return result.dynamicWindUsed
      ? THEORY_PROFILES.energyAStarDynamic
      : THEORY_PROFILES.energyAStarStatic;
  }

  return THEORY_PROFILES.energyAStarStatic;
}

function blockedCount() {
  return state.blockedCells.size;
}

function isBlocked(coord) {
  return state.blockedCells.has(coordinateKey(coord));
}

function syncWindShiftPanelState() {
  const enabled = state.environment.dynamicWindEnabled;
  elements.windShiftControls.classList.toggle("disabled", !enabled);

  elements.windShiftStep.disabled = !enabled;
  elements.windDirectionAfterShift.disabled = !enabled;
  elements.windStrengthAfterShift.disabled = !enabled;
}

function populateWindDirections() {
  elements.windDirection.innerHTML = "";
  elements.windDirectionAfterShift.innerHTML = "";

  Object.keys(DIRECTION_VECTORS).forEach((direction) => {
    const option = document.createElement("option");
    option.value = direction;
    option.textContent = direction;
    elements.windDirection.appendChild(option);

    const optionAfterShift = document.createElement("option");
    optionAfterShift.value = direction;
    optionAfterShift.textContent = direction;
    elements.windDirectionAfterShift.appendChild(optionAfterShift);
  });
}

function populateScenarioOptions() {
  elements.scenarioSelect.innerHTML = "";

  PRESET_SCENARIOS.forEach((scenario) => {
    const option = document.createElement("option");
    option.value = scenario.id;
    option.textContent = scenario.name;
    elements.scenarioSelect.appendChild(option);
  });
}

function applyControlsFromState() {
  elements.windDirection.value = state.environment.windDirection;
  elements.windStrength.value = String(state.environment.windStrength);
  elements.altitudeFactor.value = String(state.environment.mass ?? state.environment.altitudeFactor);
  elements.dynamicWindEnabled.checked = state.environment.dynamicWindEnabled;
  elements.windShiftStep.value = String(state.environment.windShiftStep);
  elements.windDirectionAfterShift.value = state.environment.windDirectionAfterShift;
  elements.windStrengthAfterShift.value = String(state.environment.windStrengthAfterShift);

  elements.windStrengthValue.textContent = state.environment.windStrength.toFixed(1);
  elements.altitudeFactorValue.textContent = (state.environment.mass ?? state.environment.altitudeFactor).toFixed(1);
  elements.windShiftStepValue.textContent = String(state.environment.windShiftStep);
  elements.windStrengthAfterShiftValue.textContent = state.environment.windStrengthAfterShift.toFixed(1);
  elements.obstaclePaintMode.checked = state.paintObstacles;
  elements.showWindFlow.checked = windFlowEnabled;

  if (!elements.obstacleDensity.value) {
    elements.obstacleDensity.value = String(DEFAULT_OBSTACLE_DENSITY);
  }

  elements.obstacleDensityValue.textContent = Number(elements.obstacleDensity.value).toFixed(2);
  syncWindShiftPanelState();
  elements.enablePathGlow.checked = state.pathGlowEnabled;
  updateWindIndicators();
  updateWindFlowVisibility();

  if (state.activeScenarioId) {
    elements.scenarioSelect.value = state.activeScenarioId;
  }
}

function altitudeBounds(grid) {
  let min = Number.POSITIVE_INFINITY;
  let max = Number.NEGATIVE_INFINITY;

  for (const row of grid) {
    for (const value of row) {
      min = Math.min(min, value);
      max = Math.max(max, value);
    }
  }

  return { min, max };
}

function altitudeColor(value, min, max) {
  const span = Math.max(0.0001, max - min);
  const ratio = (value - min) / span;
  const hue = 195 - ratio * 150;
  const lightness = 84 - ratio * 30;
  return `hsl(${hue.toFixed(0)} 68% ${lightness.toFixed(0)}%)`;
}

function pathToSet(path) {
  return new Set(path.map((coord) => coordinateKey(coord)));
}

function pathIndexMap(path) {
  const map = new Map();

  path.forEach((coord, index) => {
    map.set(coordinateKey(coord), index);
  });

  return map;
}

function pathSetCount(key, ...sets) {
  return sets.reduce((count, set) => count + (set.has(key) ? 1 : 0), 0);
}

function renderGrid() {
  if (state.altitudeGrid.length === 0) {
    return;
  }

  const rows = state.altitudeGrid.length;
  const cols = state.altitudeGrid[0].length;
  const { min, max } = altitudeBounds(state.altitudeGrid);
  const standardPath = state.results?.standard?.path ?? [];
  const energyPath = state.results?.energy?.path ?? [];
  const baselineSet = pathToSet(standardPath);
  const energySet = pathToSet(energyPath);
  const baselineIndexMap = pathIndexMap(standardPath);
  const energyIndexMap = pathIndexMap(energyPath);
  const flowStepMs = 140;
  const minFlowDurationMs = 1400;
  const standardDurationMs = Math.max(minFlowDurationMs, standardPath.length * flowStepMs);
  const energyDurationMs = Math.max(minFlowDurationMs, energyPath.length * flowStepMs);
  const overlapDurationMs = Math.max(standardDurationMs, energyDurationMs);
  const standardOffsetMs = 0;
  const energyOffsetMs = 150;

  elements.grid.style.setProperty("--grid-cols", cols);
  elements.grid.innerHTML = "";

  for (let row = 0; row < rows; row += 1) {
    for (let col = 0; col < cols; col += 1) {
      const altitude = state.altitudeGrid[row][col];
      const cell = document.createElement("button");
      const key = `${row},${col}`;

      cell.type = "button";
      cell.className = "grid-cell";
      cell.dataset.row = String(row);
      cell.dataset.col = String(col);
      cell.style.backgroundColor = altitudeColor(altitude, min, max);

      const inBaseline = baselineSet.has(key);
      const inEnergy = energySet.has(key);
      const blocked = state.blockedCells.has(key);

      if (blocked) {
        cell.classList.add("blocked");
      } else if (pathSetCount(key, baselineSet, energySet) >= 2) {
        cell.classList.add("path-overlap");
        if (state.pathGlowEnabled) {
          const indices = [
            baselineIndexMap.get(key),
            energyIndexMap.get(key),
          ].filter((value) => Number.isFinite(value));
          const overlapIndex = indices.length > 0 ? Math.min(...indices) : 0;
          cell.classList.add("path-flow", "path-overlap-flow");
          cell.style.setProperty("--flow-delay", `${overlapIndex * flowStepMs}ms`);
          cell.style.setProperty("--flow-duration", `${overlapDurationMs}ms`);
        }
      } else if (inBaseline) {
        cell.classList.add("path-standard");
        if (state.pathGlowEnabled) {
          const index = baselineIndexMap.get(key) ?? 0;
          cell.classList.add("path-flow", "path-flow-standard");
          cell.style.setProperty("--flow-delay", `${index * flowStepMs + standardOffsetMs}ms`);
          cell.style.setProperty("--flow-duration", `${standardDurationMs}ms`);
        }
      } else if (inEnergy) {
        cell.classList.add("path-energy");
        if (state.pathGlowEnabled) {
          const index = energyIndexMap.get(key) ?? 0;
          cell.classList.add("path-flow", "path-flow-energy");
          cell.style.setProperty("--flow-delay", `${index * flowStepMs + energyOffsetMs}ms`);
          cell.style.setProperty("--flow-duration", `${energyDurationMs}ms`);
        }
      }

      const isStart = sameCoord(state.start, [row, col]);
      const isEnd = sameCoord(state.end, [row, col]);

      if (isStart) {
        cell.classList.add("start");
      }

      if (isEnd) {
        cell.classList.add("end");
      }

      const marker = isStart ? "S" : isEnd ? "E" : "";
      const valueText = blocked ? "X" : altitude.toFixed(1);
      cell.innerHTML = `<span>${valueText}</span>${marker ? `<span class="marker">${marker}</span>` : ""}`;
      elements.grid.appendChild(cell);
    }
  }
}

function cardHtml(result) {
  const theory = getTheoryProfile(result);
  const gridStats = computeGridStats(state.altitudeGrid, state.blockedCells);
  const runtimeMetrics = buildRuntimeMetrics(
    result,
    gridStats,
    result.actualExecutionTime ?? result.executionTimeMs,
  );

  return `
    <div class="stat-card">
      <h3>${result.algorithm}</h3>
      <p>Objective: ${result.optimizedFor}</p>
      <p>Path distance: ${formatNumber(result.totalDistance)}</p>
      <p>Minimum energy cost: ${formatNumber(result.totalEnergyCost)}</p>
      <p>Wind energy: ${formatNumber(result.windEnergy)}</p>
      <p>Gravity energy: ${formatNumber(result.gravityEnergy)}</p>
      <p>Turning energy: ${formatNumber(result.turningEnergy)}</p>
      <p>Execution time (ms): ${formatNumber(runtimeMetrics.actualExecutionTime, 4)}</p>
      <p>Avg benchmark time (ms, ${BENCHMARK_ITERATIONS} runs): ${formatNumber(runtimeMetrics.averageExecutionTime, 4)}</p>
      <p>Expanded nodes: ${result.expandedNodes}</p>
      <p>Heuristic evaluations: ${result.heuristicEvaluations ?? "N/A"}</p>
      <p>Path steps: ${result.steps}</p>
      <p>Blocked cells: ${result.blockedCells}</p>
      <p>Dynamic wind: ${result.dynamicWindUsed ? "Enabled" : "Disabled"}</p>
      <p class="complexity-heading">Recurrence relation</p>
      <p>${theory.recurrence}</p>
      <p class="complexity-note">${theory.recurrenceNote}</p>
      <p class="complexity-heading">Time complexity</p>
      <p>Best: ${theory.bestCase}</p>
      <p>Average: ${theory.averageCase}</p>
      <p>Worst: ${theory.worstCase}</p>
      <p class="complexity-heading">Runtime stats</p>
      <p>Vertices (V): ${gridStats.vertices}</p>
      <p>Edges (E): ${gridStats.edges}</p>
      <p>Estimated workload: ${formatOps(runtimeMetrics.estimatedWorkload)}</p>
      <p>Expanded nodes: ${result.expandedNodes}</p>
      <p>Execution time (ms): ${formatNumber(runtimeMetrics.actualExecutionTime, 4)}</p>
      <p>Avg benchmark time (ms, ${BENCHMARK_ITERATIONS} runs): ${formatNumber(runtimeMetrics.averageExecutionTime, 4)}</p>
    </div>
  `;
}

function renderStatsCards() {
  if (!state.results) {
    elements.statsCards.innerHTML = "<p class=\"muted-text\">Run all algorithms to view metrics.</p>";
    return;
  }

  elements.statsCards.innerHTML = `${cardHtml(state.results.standard)}${cardHtml(state.results.energy)}`;
}

function renderComparisonTable() {
  if (!state.results) {
    elements.comparisonTable.innerHTML = "";
    return;
  }

  const standard = state.results.standard;
  const energy = state.results.energy;
  const standardTheory = getTheoryProfile(standard);
  const energyTheory = getTheoryProfile(energy);
  const gridStats = computeGridStats(state.altitudeGrid, state.blockedCells);
  const standardMetrics = buildRuntimeMetrics(
    standard,
    gridStats,
    standard.actualExecutionTime ?? standard.executionTimeMs,
  );
  const energyMetrics = buildRuntimeMetrics(
    energy,
    gridStats,
    energy.actualExecutionTime ?? energy.executionTimeMs,
  );
  const bestEnergy = Math.min(standard.totalEnergyCost, energy.totalEnergyCost);
  const bestTime = Math.min(
    standardMetrics.actualExecutionTime,
    energyMetrics.actualExecutionTime,
  );
  const bestExpanded = Math.min(standard.expandedNodes, energy.expandedNodes);
  const costBaseline = standard.totalEnergyCost;

  const energyBadgeStandard = standard.totalEnergyCost === bestEnergy ? metricBadge("Best") : "";
  const energyBadgeEnergy = energy.totalEnergyCost === bestEnergy ? metricBadge("Best") : "";
  const timeBadgeStandard = standardMetrics.actualExecutionTime === bestTime ? metricBadge("Fast") : "";
  const timeBadgeEnergy = energyMetrics.actualExecutionTime === bestTime ? metricBadge("Fast") : "";
  const expandedBadgeStandard = standard.expandedNodes === bestExpanded ? metricBadge("Light") : "";
  const expandedBadgeEnergy = energy.expandedNodes === bestExpanded ? metricBadge("Light") : "";

  const standardCostImprove = costBaseline > 0 ? 0 : null;
  const energyCostImprove = costBaseline > 0 ? (costBaseline - energy.totalEnergyCost) / costBaseline : null;

  const standardUniqueRatio = gridStats.vertices > 0
    ? standard.uniqueVisitedNodes / gridStats.vertices
    : null;
  const energyUniqueRatio = gridStats.vertices > 0
    ? energy.uniqueVisitedNodes / gridStats.vertices
    : null;

  const standardEfficiency = standardMetrics.actualExecutionTime > 0
    ? standard.expandedNodes / standardMetrics.actualExecutionTime
    : null;
  const energyEfficiency = energyMetrics.actualExecutionTime > 0
    ? energy.expandedNodes / energyMetrics.actualExecutionTime
    : null;

  const standardExplorationEfficiency = standard.expandedNodes > 0 ? standard.steps / standard.expandedNodes : null;
  const energyExplorationEfficiency = energy.expandedNodes > 0 ? energy.steps / energy.expandedNodes : null;


  elements.comparisonTable.innerHTML = `
    <tr class="table-section">
      <td colspan="3">Core Metrics</td>
    </tr>
    <tr>
      <td>Objective value</td>
      <td>${formatNumber(standard.objectiveCost)}</td>
      <td>${formatNumber(energy.objectiveCost)}</td>
    </tr>
    <tr>
      <td>Optimization target</td>
      <td>${standard.optimizedFor}</td>
      <td>${energy.optimizedFor}</td>
    </tr>
    <tr>
      <td>Path distance</td>
      <td>${formatNumber(standard.totalDistance)}</td>
      <td>${formatNumber(energy.totalDistance)}</td>
    </tr>
    <tr>
      <td>Minimum energy cost</td>
      <td>${formatNumber(standard.totalEnergyCost)} ${energyBadgeStandard}</td>
      <td>${formatNumber(energy.totalEnergyCost)} ${energyBadgeEnergy}</td>
    </tr>
    <tr>
      <td>Wind energy</td>
      <td>${formatNumber(standard.windEnergy)}</td>
      <td>${formatNumber(energy.windEnergy)}</td>
    </tr>
    <tr>
      <td>Gravity energy</td>
      <td>${formatNumber(standard.gravityEnergy)}</td>
      <td>${formatNumber(energy.gravityEnergy)}</td>
    </tr>
    <tr>
      <td>Turning energy</td>
      <td>${formatNumber(standard.turningEnergy)}</td>
      <td>${formatNumber(energy.turningEnergy)}</td>
    </tr>
    <tr>
      <td>Execution time (ms)</td>
      <td>${formatNumber(standardMetrics.actualExecutionTime, 4)} ${timeBadgeStandard}</td>
      <td>${formatNumber(energyMetrics.actualExecutionTime, 4)} ${timeBadgeEnergy}</td>
    </tr>
    <tr>
      <td>Avg benchmark time (ms, ${BENCHMARK_ITERATIONS} runs)</td>
      <td>${formatNumber(standardMetrics.averageExecutionTime, 4)}</td>
      <td>${formatNumber(energyMetrics.averageExecutionTime, 4)}</td>
    </tr>
    <tr>
      <td>Expanded nodes</td>
      <td>${standard.expandedNodes} ${expandedBadgeStandard}</td>
      <td>${energy.expandedNodes} ${expandedBadgeEnergy}</td>
    </tr>
    <tr>
      <td>Unique visited nodes</td>
      <td>${standard.uniqueVisitedNodes}</td>
      <td>${energy.uniqueVisitedNodes}</td>
    </tr>
    <tr>
      <td>Relaxation operations</td>
      <td>${formatOps(standard.relaxationOperations)}</td>
      <td>${formatOps(energy.relaxationOperations)}</td>
    </tr>
    <tr>
      <td>Heuristic evaluations</td>
      <td>N/A</td>
      <td>${formatOps(energy.heuristicEvaluations ?? 0)}</td>
    </tr>
    <tr>
      <td>Path steps</td>
      <td>${standard.steps}</td>
      <td>${energy.steps}</td>
    </tr>
    <tr>
      <td>Blocked cells</td>
      <td>${standard.blockedCells}</td>
      <td>${energy.blockedCells}</td>
    </tr>
    <tr class="table-section">
      <td colspan="3">Graph & Runtime Metrics</td>
    </tr>
    <tr>
      <td>Vertices (V)</td>
      <td>${gridStats.vertices}</td>
      <td>${gridStats.vertices}</td>
    </tr>
    <tr>
      <td>Edges (E)</td>
      <td>${gridStats.edges}</td>
      <td>${gridStats.edges}</td>
    </tr>
    <tr>
      <td>Complexity formula</td>
      <td>${standardMetrics.theoreticalComplexity}</td>
      <td>${energyMetrics.theoreticalComplexity}</td>
    </tr>
    <tr>
      <td>Runtime formula used</td>
      <td>${standardMetrics.runtimeFormula}</td>
      <td>${energyMetrics.runtimeFormula}</td>
    </tr>
    <tr>
      <td>Estimated computational workload <span class="metric-note" title="Theoretical complexity represents asymptotic growth, while workload estimates operation scale using current graph size.">i</span></td>
      <td>${formatOps(standardMetrics.estimatedWorkload)}</td>
      <td>${formatOps(energyMetrics.estimatedWorkload)}</td>
    </tr>
    <tr>
      <td>Unique visited ratio</td>
      <td>${formatPercent(standardUniqueRatio)}</td>
      <td>${formatPercent(energyUniqueRatio)}</td>
    </tr>
    <tr>
      <td>Efficiency score (nodes/ms)</td>
      <td>${formatRatio(standardEfficiency)}</td>
      <td>${formatRatio(energyEfficiency)}</td>
    </tr>
    <tr>
      <td>Exploration efficiency (steps/expanded)</td>
      <td>${formatRatio(standardExplorationEfficiency)}</td>
      <td>${formatRatio(energyExplorationEfficiency)}</td>
    </tr>
    <tr>
      <td>Cost improvement % (vs standard)</td>
      <td>${formatPercent(standardCostImprove)}</td>
      <td>${formatPercent(energyCostImprove)}</td>
    </tr>
    <tr class="table-section">
      <td colspan="3">Theory References</td>
    </tr>
    <tr>
      <td>Recurrence relation</td>
      <td>${standardTheory.recurrence}</td>
      <td>${energyTheory.recurrence}</td>
    </tr>
    <tr>
      <td>Recurrence applicability</td>
      <td>${standardTheory.recurrenceNote}</td>
      <td>${energyTheory.recurrenceNote}</td>
    </tr>
    <tr>
      <td>Best-case time complexity</td>
      <td>${standardTheory.bestCase}</td>
      <td>${energyTheory.bestCase}</td>
    </tr>
    <tr>
      <td>Average-case time complexity</td>
      <td>${standardTheory.averageCase}</td>
      <td>${energyTheory.averageCase}</td>
    </tr>
    <tr>
      <td>Worst-case time complexity</td>
      <td>${standardTheory.worstCase}</td>
      <td>${energyTheory.worstCase}</td>
    </tr>
  `;
}

function formatPath(path) {
  if (!path || path.length === 0) {
    return "No path";
  }

  if (path.length <= 14) {
    return path.map((coord) => formatCoord(coord)).join(" -> ");
  }

  const head = path.slice(0, 7).map((coord) => formatCoord(coord)).join(" -> ");
  const tail = path.slice(-7).map((coord) => formatCoord(coord)).join(" -> ");
  return `${head} -> ... -> ${tail}`;
}

function renderPathSummary() {
  if (!state.start || !state.end) {
    elements.pathSummary.textContent = "Select both start and end nodes.";
    return;
  }

  if (!state.results) {
    elements.pathSummary.textContent = `Start: ${formatCoord(state.start)}\nEnd: ${formatCoord(state.end)}`;
    return;
  }

  const standardPath = state.results.standard.path;
  const energyPath = state.results.energy.path;
  const pathChanged = JSON.stringify(standardPath) !== JSON.stringify(energyPath);
  const windMode = state.environment.dynamicWindEnabled
    ? `Dynamic (shift at step ${state.environment.windShiftStep}: ${state.environment.windDirection}/${state.environment.windStrength.toFixed(1)} -> ${state.environment.windDirectionAfterShift}/${state.environment.windStrengthAfterShift.toFixed(1)})`
    : `Static (${state.environment.windDirection}/${state.environment.windStrength.toFixed(1)})`;

  elements.pathSummary.textContent =
    `Start: ${formatCoord(state.start)}\n` +
    `End: ${formatCoord(state.end)}\n` +
    `Blocked cells: ${blockedCount()}\n` +
    `Wind mode: ${windMode}\n` +
    `Paths differ: ${pathChanged ? "Yes" : "No"}\n\n` +
    `Standard path:\n${formatPath(standardPath)}\n\n` +
    `Energy-Aware A* path:\n${formatPath(energyPath)}`;

}

function describeMetricDifference(metricName, standardValue, candidateValue, options = {}) {
  const { unit = "", digits = 2, lowerIsBetter = true } = options;

  if (!Number.isFinite(standardValue) || !Number.isFinite(candidateValue)) {
    return `${metricName}: could not be compared for this run.`;
  }

  const delta = candidateValue - standardValue;
  if (Math.abs(delta) < 0.000001) {
    return `${metricName}: both values are effectively equal (${formatNumber(candidateValue, digits)}${unit}).`;
  }

  const directionWord = delta < 0 ? "lower" : "higher";
  const betterWord = lowerIsBetter ? (delta < 0 ? "better" : "worse") : delta > 0 ? "better" : "worse";
  const deltaPercent = percentDelta(standardValue, candidateValue);
  const percentText = deltaPercent === null ? "" : ` (${formatSignedNumber(deltaPercent, 1)}%)`;

  return (
    `${metricName}: candidate is ${formatNumber(Math.abs(delta), digits)}${unit} ${directionWord}` +
    `${percentText}, so it is ${betterWord} for this metric.`
  );
}

function renderResultExplanation() {
  if (!state.results) {
    elements.resultExplanation.innerHTML =
      "<p class=\"muted-text\">Run all algorithms to generate a case-specific explanation of the differences.</p>";
    return;
  }

  const standard = state.results.standard;
  const energy = state.results.energy;

  if (standard.path.length === 0 || energy.path.length === 0) {
    elements.resultExplanation.innerHTML =
      "<h3>Case-Specific Explanation</h3><p>At least one algorithm could not find a feasible route. In this setup, obstacles or environmental penalties likely disconnected the destination.</p>";
    return;
  }

  const pathChanged = JSON.stringify(standard.path) !== JSON.stringify(energy.path);
  const energyDelta = energy.totalEnergyCost - standard.totalEnergyCost;
  const windDelta = energy.windEnergy - standard.windEnergy;
  const gravityDelta = energy.gravityEnergy - standard.gravityEnergy;
  const turningDelta = energy.turningEnergy - standard.turningEnergy;
  const dynamicWindText = state.environment.dynamicWindEnabled
    ? "Dynamic wind is enabled, so the energy-aware solver plans while accounting for the wind shift during traversal."
    : "Wind is static in this run, so differences are caused by directional wind resistance and altitude trade-offs.";

  let interpretation = "In this run, Energy-Aware A* improved the targeted energy objective.";
  if (energyDelta > 0.000001) {
    interpretation = "In this run, Energy-Aware A* produced a higher evaluated energy cost than standard Dijkstra, which suggests this case is strongly constrained by map geometry or obstacle layout.";
  } else if (Math.abs(energyDelta) <= 0.000001) {
    interpretation = "In this run, both methods produced nearly identical energy cost, indicating the distance-minimizing route was also energy-efficient.";
  }

  const windInterpretation =
    windDelta < -0.000001
      ? "Wind exposure is lower for Energy-Aware A*, so it avoided headwind-heavy moves."
      : windDelta > 0.000001
        ? "Wind exposure is higher for Energy-Aware A*, meaning it traded wind cost for gains elsewhere."
        : "Wind exposure is nearly the same for standard and energy-aware runs.";

  const gravityInterpretation =
    gravityDelta < -0.000001
      ? "Gravity energy is lower for Energy-Aware A*, so it selected a smoother climb profile."
      : gravityDelta > 0.000001
        ? "Gravity energy is higher for Energy-Aware A*, indicating it accepted more climb to reduce other costs."
        : "Gravity energy is nearly identical for standard and energy-aware runs.";

  const turningInterpretation =
    turningDelta < -0.000001
      ? "Turning energy is lower for Energy-Aware A*, meaning it favored smoother heading changes."
      : turningDelta > 0.000001
        ? "Turning energy is higher for Energy-Aware A*, meaning it accepted sharper turns to reduce other costs."
        : "Turning energy is nearly identical for standard and energy-aware runs.";

  const pathNarrative = pathChanged
    ? "Routes differ, which confirms that environmental energy factors changed path selection beyond pure distance minimization."
    : "Routes are the same, so distance-minimizing and energy-aware optimization aligned in this setup.";

  elements.resultExplanation.innerHTML =
    "<h3>Case-Specific Explanation</h3>" +
    `<p>${pathNarrative}</p>` +
    "<ul>" +
    `<li>${describeMetricDifference("Minimum energy cost", standard.totalEnergyCost, energy.totalEnergyCost, { lowerIsBetter: true })}</li>` +
    `<li>${describeMetricDifference("Path distance", standard.totalDistance, energy.totalDistance, { lowerIsBetter: true })}</li>` +
    `<li>${describeMetricDifference("Wind energy", standard.windEnergy, energy.windEnergy, { lowerIsBetter: true })}</li>` +
    `<li>${describeMetricDifference("Gravity energy", standard.gravityEnergy, energy.gravityEnergy, { lowerIsBetter: true })}</li>` +
    `<li>${describeMetricDifference("Turning energy", standard.turningEnergy, energy.turningEnergy, { lowerIsBetter: true })}</li>` +
    `<li>${describeMetricDifference("Path steps", standard.steps, energy.steps, { lowerIsBetter: true, digits: 0 })}</li>` +
    "</ul>" +
    `<p>${windInterpretation}</p>` +
    `<p>${gravityInterpretation}</p>` +
    `<p>${turningInterpretation}</p>` +
    `<p>${dynamicWindText}</p>` +
    `<p>${interpretation}</p>`;
}

function renderAll() {
  renderGrid();
  renderStatsCards();
  renderComparisonTable();
  renderPathSummary();
  renderResultExplanation();
}

function clearTourFocus() {
  if (!state.tourFocusedElement) {
    return;
  }

  state.tourFocusedElement.classList.remove("tour-focus");
  state.tourFocusedElement = null;
}

function highlightTourTarget(selector) {
  clearTourFocus();

  const target = document.querySelector(selector);
  if (!target) {
    return;
  }

  state.tourFocusedElement = target;
  target.classList.add("tour-focus");
  target.scrollIntoView({ behavior: "smooth", block: "center", inline: "nearest" });
}

function renderTourStep() {
  const step = TOUR_STEPS[state.tourStepIndex];
  if (!step) {
    return;
  }

  elements.tourStepCounter.textContent = `Step ${state.tourStepIndex + 1} of ${TOUR_STEPS.length}`;
  elements.tourTitle.textContent = step.title;
  elements.tourBody.textContent = step.body;

  elements.tourBackBtn.disabled = state.tourStepIndex === 0;
  elements.tourNextBtn.textContent = state.tourStepIndex === TOUR_STEPS.length - 1 ? "Finish" : "Next";

  highlightTourTarget(step.selector);
}

function openTour(startIndex = 0) {
  state.tourActive = true;
  state.tourStepIndex = Math.min(Math.max(startIndex, 0), TOUR_STEPS.length - 1);
  elements.tourOverlay.classList.remove("hidden");
  renderTourStep();
}

function closeTour(options = {}) {
  const { rememberDismissed = false, statusMessage = "" } = options;

  state.tourActive = false;
  elements.tourOverlay.classList.add("hidden");
  clearTourFocus();

  if (rememberDismissed) {
    try {
      localStorage.setItem(TOUR_DISMISSED_STORAGE_KEY, "1");
    } catch {
      // Ignore storage write failures and continue.
    }
  }

  if (statusMessage) {
    setStatus(statusMessage);
  }
}

function maybeAutoStartTour() {
  let dismissed = false;

  try {
    dismissed = localStorage.getItem(TOUR_DISMISSED_STORAGE_KEY) === "1";
  } catch {
    dismissed = false;
  }

  if (!dismissed) {
    openTour(0);
  }
}

function runAlgorithms() {
  if (!state.start || !state.end) {
    setStatus("Choose start and end nodes before running.");
    return;
  }

  if (isBlocked(state.start) || isBlocked(state.end)) {
    setStatus("Start/End cannot be blocked. Clear the obstacle on those nodes first.");
    return;
  }

  const model = {
    altitudeGrid: state.altitudeGrid,
    start: state.start,
    end: state.end,
    environment: state.environment,
    blockedSet: state.blockedCells,
  };

  const standardAvg = benchmarkStandard(model);
  const energyAvg = benchmarkEnergyAwareAStar(model);

  const standardStart = performance.now();
  const standard = runStandardDijkstra(model);
  const standardEnd = performance.now();

  const energyStart = performance.now();
  const energy = runEnergyAwareAStar(model);
  const energyEnd = performance.now();


  standard.actualExecutionTime = standardEnd - standardStart;
  energy.actualExecutionTime = energyEnd - energyStart;
  standard.averageExecutionTime = standardAvg;
  energy.averageExecutionTime = energyAvg;
  standard.executionTime = standard.actualExecutionTime;
  energy.executionTime = energy.actualExecutionTime;

  state.results = {
    standard,
    energy,
  };

  renderAll();

  // Smooth scroll to results section
  const resultElement = elements.statsCards;
  if (resultElement && resultElement.offsetParent !== null) {
    resultElement.scrollIntoView({ behavior: "smooth", block: "start" });
  }

  const pathChanged = JSON.stringify(standard.path) !== JSON.stringify(energy.path);

  if (standard.path.length === 0 || energy.path.length === 0) {
    setStatus("No feasible route found. Remove some obstacles or adjust parameters.");
    return;
  }

  setStatus(
    pathChanged
      ? "Algorithms completed. Energy-aware routing selected a different minimum-energy trajectory."
      : "Algorithms completed. Baseline and energy-aware routing match in this setup.",
  );
}

async function generateAnalysisReport() {
  const payload = buildReportPayload();

  if (!payload) {
    setStatus("Run all algorithms first, then generate a report.");
    return;
  }

  const button = elements.generateReportBtn;
  const originalLabel = button.textContent;
  button.disabled = true;
  button.textContent = "Generating...";

  try {
    const response = await fetch(REPORT_SERVER_URL, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify(payload),
    });

    if (!response.ok) {
      const message = await response.text();
      throw new Error(message || "Report server returned an error.");
    }

    const result = await response.json();
    const folder = result?.folder ?? "reports";
    setStatus(`Report generated: ${folder}`);
  } catch (error) {
    const message = error instanceof Error ? error.message : "Unknown error.";
    setStatus(`Report generation failed. ${message}`);
  } finally {
    button.disabled = false;
    button.textContent = originalLabel;
  }
}

function ensureEndpointsUnblocked(candidate) {
  if (state.start) {
    candidate.delete(coordinateKey(state.start));
  }

  if (state.end) {
    candidate.delete(coordinateKey(state.end));
  }
}

function tryBuildObstacleSet(density) {
  const rows = state.altitudeGrid.length;
  const cols = state.altitudeGrid[0].length;
  const maxAttempts = 14;

  let fallback = new Set();

  for (let attempt = 0; attempt < maxAttempts; attempt += 1) {
    const candidate = new Set();

    for (let row = 0; row < rows; row += 1) {
      for (let col = 0; col < cols; col += 1) {
        if (Math.random() < density) {
          candidate.add(`${row},${col}`);
        }
      }
    }

    ensureEndpointsUnblocked(candidate);

    const testModel = {
      altitudeGrid: state.altitudeGrid,
      start: state.start,
      end: state.end,
      environment: state.environment,
      blockedSet: candidate,
    };

    const sanityRoute = runStandardDijkstra(testModel);
    if (sanityRoute.path.length > 0) {
      return candidate;
    }

    if (fallback.size === 0 || candidate.size < fallback.size) {
      fallback = candidate;
    }
  }

  return fallback;
}

function randomizeObstacles() {
  if (!state.start || !state.end) {
    setStatus("Set start and end first, then generate random obstacles.");
    return;
  }

  const density = Number(elements.obstacleDensity.value);
  const generated = tryBuildObstacleSet(density);

  state.blockedCells = generated;
  state.results = null;
  renderAll();

  if (generated.size === 0) {
    setStatus("No valid obstacle map found with this density. Try lower density.");
    return;
  }

  const quickCheck = runStandardDijkstra({
    altitudeGrid: state.altitudeGrid,
    start: state.start,
    end: state.end,
    environment: state.environment,
    blockedSet: generated,
  });

  if (quickCheck.path.length === 0) {
    setStatus(`Generated ${generated.size} obstacles, but grid may be disconnected. Reduce density.`);
    return;
  }

  setStatus(`Generated ${generated.size} obstacles. Run all algorithms to compare.`);
}

function clearObstacles() {
  state.blockedCells = new Set();
  state.results = null;
  renderAll();
  setStatus("All obstacles cleared.");
}

function toggleObstacleAt(coord) {
  const key = coordinateKey(coord);

  if (sameCoord(coord, state.start) || sameCoord(coord, state.end)) {
    setStatus("Cannot place an obstacle on start or end node.");
    return;
  }

  if (state.blockedCells.has(key)) {
    state.blockedCells.delete(key);
  } else {
    state.blockedCells.add(key);
  }

  state.results = null;
  renderAll();
  setStatus(`Obstacle edit applied. Active blocked cells: ${blockedCount()}.`);
}

function randomizeAltitude() {
  const rows = state.altitudeGrid.length || 8;
  const cols = state.altitudeGrid[0]?.length || 8;

  state.altitudeGrid = createRandomAltitudeGrid(rows, cols);
  state.blockedCells = new Set();
  state.results = null;

  if (!state.start) {
    state.start = [0, 0];
  }

  if (!state.end) {
    state.end = [rows - 1, cols - 1];
  }

  if (sameCoord(state.start, state.end)) {
    state.end = [rows - 1, cols - 1];
  }

  renderAll();
  setStatus("Generated a new altitude map. Run all algorithms to compare routes.");
}

function clearSelection() {
  state.start = null;
  state.end = null;
  state.results = null;
  renderAll();
  setStatus("Start and end selection cleared.");
}

function loadScenario(id) {
  const scenario = getScenarioById(id);

  if (!scenario) {
    return;
  }

  state.activeScenarioId = scenario.id;
  state.altitudeGrid = cloneGrid(scenario.altitudeGrid);
  state.start = [...scenario.start];
  state.end = [...scenario.end];
  state.environment = createEnvironment(scenario.environment);
  state.blockedCells = new Set((scenario.obstacles ?? []).map((coord) => coordinateKey(coord)));
  state.paintObstacles = false;
  state.results = null;

  elements.scenarioDescription.textContent = scenario.description;
  applyControlsFromState();
  renderAll();
  setStatus(`Loaded ${scenario.name} with ${blockedCount()} obstacles. Click Run All Algorithms.`);
}

function onGridClick(event) {
  const targetCell = event.target.closest(".grid-cell");

  if (!targetCell) {
    return;
  }

  const row = Number(targetCell.dataset.row);
  const col = Number(targetCell.dataset.col);
  const clickedCoord = [row, col];

  if (state.paintObstacles && event.shiftKey) {
    toggleObstacleAt(clickedCoord);
    return;
  }

  if (state.paintObstacles && !event.shiftKey) {
    state.paintObstacles = false;
    elements.obstaclePaintMode.checked = false;
    setStatus("Obstacle paint mode was enabled. It is now off, so you can set start/end nodes.");
  }

  if (isBlocked(clickedCoord)) {
    setStatus("This node is blocked. Disable obstacle paint mode or clear this obstacle first.");
    return;
  }

  if (!state.start || (state.start && state.end)) {
    state.start = clickedCoord;
    state.end = null;
    state.results = null;
    setStatus(`Start selected at ${formatCoord(clickedCoord)}. Select an end node.`);
    renderAll();
    return;
  }

  if (sameCoord(state.start, clickedCoord)) {
    setStatus("End node must be different from the start node.");
    return;
  }

  state.end = clickedCoord;
  state.results = null;
  renderAll();
  setStatus(`End selected at ${formatCoord(clickedCoord)}. Click Run All Algorithms.`);
}

function bindEvents() {
  elements.startTourBtn.addEventListener("click", () => {
    openTour(0);
    setStatus("Guided tour started. Use Next, Back, or Skip.");
  });

  elements.windDirection.addEventListener("change", () => {
    state.environment.windDirection = elements.windDirection.value;
    updateWindIndicators();
  });

  elements.windStrength.addEventListener("input", () => {
    state.environment.windStrength = Number(elements.windStrength.value);
    elements.windStrengthValue.textContent = state.environment.windStrength.toFixed(1);
    updateWindIndicators();
  });

  elements.altitudeFactor.addEventListener("input", () => {
    state.environment.mass = Number(elements.altitudeFactor.value);
    elements.altitudeFactorValue.textContent = state.environment.mass.toFixed(1);
  });

  elements.dynamicWindEnabled.addEventListener("change", () => {
    state.environment.dynamicWindEnabled = elements.dynamicWindEnabled.checked;
    syncWindShiftPanelState();
    state.results = null;
    renderPathSummary();
    updateWindIndicators();
  });

  elements.windShiftStep.addEventListener("input", () => {
    state.environment.windShiftStep = Number(elements.windShiftStep.value);
    elements.windShiftStepValue.textContent = String(state.environment.windShiftStep);
    updateWindIndicators();
  });

  elements.windDirectionAfterShift.addEventListener("change", () => {
    state.environment.windDirectionAfterShift = elements.windDirectionAfterShift.value;
    updateWindIndicators();
  });

  elements.windStrengthAfterShift.addEventListener("input", () => {
    state.environment.windStrengthAfterShift = Number(elements.windStrengthAfterShift.value);
    elements.windStrengthAfterShiftValue.textContent = state.environment.windStrengthAfterShift.toFixed(1);
    updateWindIndicators();
  });

  elements.showWindFlow.addEventListener("change", () => {
    windFlowEnabled = elements.showWindFlow.checked;
    updateWindFlowVisibility();
  });

  elements.obstaclePaintMode.addEventListener("change", () => {
    state.paintObstacles = elements.obstaclePaintMode.checked;
    setStatus(
      state.paintObstacles
        ? "Obstacle paint mode enabled. Use Shift + Click on grid cells to paint obstacles."
        : "Obstacle paint mode disabled.",
    );
  });

  elements.obstacleDensity.addEventListener("input", () => {
    elements.obstacleDensityValue.textContent = Number(elements.obstacleDensity.value).toFixed(2);
  });

  elements.loadScenarioBtn.addEventListener("click", () => {
    loadScenario(elements.scenarioSelect.value);
  });

  elements.enablePathGlow.addEventListener("change", () => {
    state.pathGlowEnabled = elements.enablePathGlow.checked;
    renderGrid();
  });

  elements.randomObstacleBtn.addEventListener("click", randomizeObstacles);
  elements.clearObstacleBtn.addEventListener("click", clearObstacles);

  elements.runBtn.addEventListener("click", runAlgorithms);
  elements.generateReportBtn.addEventListener("click", generateAnalysisReport);
  elements.randomizeBtn.addEventListener("click", randomizeAltitude);
  elements.resetSelectionBtn.addEventListener("click", clearSelection);
  elements.grid.addEventListener("click", onGridClick);

  elements.tourBackBtn.addEventListener("click", () => {
    if (state.tourStepIndex === 0) {
      return;
    }

    state.tourStepIndex -= 1;
    renderTourStep();
  });

  elements.tourNextBtn.addEventListener("click", () => {
    if (state.tourStepIndex >= TOUR_STEPS.length - 1) {
      closeTour({ statusMessage: "Guided tour completed. Run algorithms to view case analysis." });
      return;
    }

    state.tourStepIndex += 1;
    renderTourStep();
  });

  elements.tourSkipBtn.addEventListener("click", () => {
    closeTour({
      rememberDismissed: true,
      statusMessage: "Guided tour skipped. Click Start Guided Tour anytime to reopen.",
    });
  });

  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape" && state.tourActive) {
      closeTour({ statusMessage: "Guided tour closed." });
    }
  });
}

function init() {
  populateWindDirections();
  populateScenarioOptions();
  bindEvents();

  const firstScenario = PRESET_SCENARIOS[0];
  loadScenario(firstScenario.id);
  maybeAutoStartTour();
}

init();
