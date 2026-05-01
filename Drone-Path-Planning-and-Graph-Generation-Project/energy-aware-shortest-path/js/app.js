import {
  DEFAULT_ENVIRONMENT,
  DIRECTION_VECTORS,
  coordinateKey,
  createRandomAltitudeGrid,
  formatCoord,
  runModifiedDijkstra,
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
  obstaclePaintMode: document.getElementById("obstaclePaintMode"),
  obstacleDensity: document.getElementById("obstacleDensity"),
  obstacleDensityValue: document.getElementById("obstacleDensityValue"),
  randomObstacleBtn: document.getElementById("randomObstacleBtn"),
  clearObstacleBtn: document.getElementById("clearObstacleBtn"),
  randomizeBtn: document.getElementById("randomizeBtn"),
  resetSelectionBtn: document.getElementById("resetSelectionBtn"),
  runBtn: document.getElementById("runBtn"),
  grid: document.getElementById("grid"),
  statsCards: document.getElementById("statsCards"),
  comparisonTable: document.querySelector("#comparisonTable tbody"),
  pathSummary: document.getElementById("pathSummary"),
  resultExplanation: document.getElementById("resultExplanation"),
  statusLine: document.getElementById("statusLine"),
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
const TOUR_STEPS = [
  {
    selector: ".hero",
    title: "Project Overview",
    body: "This header summarizes the goal: compare shortest-distance routing against energy-aware routing under wind and altitude effects.",
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
    body: "Sets the wind vector direction that influences movement cost in the modified algorithm.",
  },
  {
    selector: "#windStrength",
    title: "Wind Strength",
    body: "Controls how strongly wind affects energy cost. Higher values penalize moving against wind more.",
  },
  {
    selector: "#altitudeFactor",
    title: "Altitude Cost Factor",
    body: "Scales how expensive elevation changes are. Higher values encourage flatter routes.",
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
    title: "Run Both Algorithms",
    body: "Executes standard Dijkstra and energy-aware Dijkstra for the current grid and settings.",
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
    body: "Lists start/end, wind mode, whether paths differ, and a condensed route string for both algorithms.",
  },
  {
    selector: "#resultExplanation",
    title: "Case Explanation",
    body: "After each run, this section explains why modified Dijkstra behaved differently in this exact scenario.",
  },
];

function createEnvironment(overrides = {}) {
  return {
    ...DEFAULT_ENVIRONMENT,
    ...overrides,
  };
}

const state = {
  altitudeGrid: [],
  start: null,
  end: null,
  environment: createEnvironment(),
  blockedCells: new Set(),
  paintObstacles: false,
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

function setStatus(text) {
  elements.statusLine.textContent = text;
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
  elements.altitudeFactor.value = String(state.environment.altitudeFactor);
  elements.dynamicWindEnabled.checked = state.environment.dynamicWindEnabled;
  elements.windShiftStep.value = String(state.environment.windShiftStep);
  elements.windDirectionAfterShift.value = state.environment.windDirectionAfterShift;
  elements.windStrengthAfterShift.value = String(state.environment.windStrengthAfterShift);

  elements.windStrengthValue.textContent = state.environment.windStrength.toFixed(1);
  elements.altitudeFactorValue.textContent = state.environment.altitudeFactor.toFixed(1);
  elements.windShiftStepValue.textContent = String(state.environment.windShiftStep);
  elements.windStrengthAfterShiftValue.textContent = state.environment.windStrengthAfterShift.toFixed(1);
  elements.obstaclePaintMode.checked = state.paintObstacles;

  if (!elements.obstacleDensity.value) {
    elements.obstacleDensity.value = String(DEFAULT_OBSTACLE_DENSITY);
  }

  elements.obstacleDensityValue.textContent = Number(elements.obstacleDensity.value).toFixed(2);
  syncWindShiftPanelState();

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

function renderGrid() {
  if (state.altitudeGrid.length === 0) {
    return;
  }

  const rows = state.altitudeGrid.length;
  const cols = state.altitudeGrid[0].length;
  const { min, max } = altitudeBounds(state.altitudeGrid);
  const baselineSet = pathToSet(state.results?.standard?.path ?? []);
  const energySet = pathToSet(state.results?.modified?.path ?? []);

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
      } else if (inBaseline && inEnergy) {
        cell.classList.add("path-overlap");
      } else if (inBaseline) {
        cell.classList.add("path-standard");
      } else if (inEnergy) {
        cell.classList.add("path-energy");
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
  return `
    <div class="stat-card">
      <h3>${result.algorithm}</h3>
      <p>Objective: ${result.optimizedFor}</p>
      <p>Total distance: ${formatNumber(result.totalDistance)}</p>
      <p>Total energy cost: ${formatNumber(result.totalEnergyCost)}</p>
      <p>Wind penalty: ${formatNumber(result.windPenalty)}</p>
      <p>Altitude penalty: ${formatNumber(result.altitudePenalty)}</p>
      <p>Execution time (ms): ${formatNumber(result.executionTimeMs, 4)}</p>
      <p>Expanded nodes: ${result.expandedNodes}</p>
      <p>Path steps: ${result.steps}</p>
      <p>Blocked cells: ${result.blockedCells}</p>
      <p>Dynamic wind: ${result.dynamicWindUsed ? "Enabled" : "Disabled"}</p>
    </div>
  `;
}

function renderStatsCards() {
  if (!state.results) {
    elements.statsCards.innerHTML = "<p class=\"muted-text\">Run both algorithms to view metrics.</p>";
    return;
  }

  elements.statsCards.innerHTML = `${cardHtml(state.results.standard)}${cardHtml(state.results.modified)}`;
}

function renderComparisonTable() {
  if (!state.results) {
    elements.comparisonTable.innerHTML = "";
    return;
  }

  const standard = state.results.standard;
  const modified = state.results.modified;

  elements.comparisonTable.innerHTML = `
    <tr>
      <td>Objective value</td>
      <td>${formatNumber(standard.objectiveCost)}</td>
      <td>${formatNumber(modified.objectiveCost)}</td>
    </tr>
    <tr>
      <td>Optimized objective</td>
      <td>${standard.optimizedFor}</td>
      <td>${modified.optimizedFor}</td>
    </tr>
    <tr>
      <td>Total distance</td>
      <td>${formatNumber(standard.totalDistance)}</td>
      <td>${formatNumber(modified.totalDistance)}</td>
    </tr>
    <tr>
      <td>Total energy cost</td>
      <td>${formatNumber(standard.totalEnergyCost)}</td>
      <td>${formatNumber(modified.totalEnergyCost)}</td>
    </tr>
    <tr>
      <td>Wind penalty</td>
      <td>${formatNumber(standard.windPenalty)}</td>
      <td>${formatNumber(modified.windPenalty)}</td>
    </tr>
    <tr>
      <td>Altitude penalty</td>
      <td>${formatNumber(standard.altitudePenalty)}</td>
      <td>${formatNumber(modified.altitudePenalty)}</td>
    </tr>
    <tr>
      <td>Execution time (ms)</td>
      <td>${formatNumber(standard.executionTimeMs, 4)}</td>
      <td>${formatNumber(modified.executionTimeMs, 4)}</td>
    </tr>
    <tr>
      <td>Expanded nodes</td>
      <td>${standard.expandedNodes}</td>
      <td>${modified.expandedNodes}</td>
    </tr>
    <tr>
      <td>Path steps</td>
      <td>${standard.steps}</td>
      <td>${modified.steps}</td>
    </tr>
    <tr>
      <td>Blocked cells</td>
      <td>${standard.blockedCells}</td>
      <td>${modified.blockedCells}</td>
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
  const modifiedPath = state.results.modified.path;
  const pathChanged = JSON.stringify(standardPath) !== JSON.stringify(modifiedPath);
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
    `Energy-aware path:\n${formatPath(modifiedPath)}`;
}

function describeMetricDifference(metricName, standardValue, modifiedValue, options = {}) {
  const { unit = "", digits = 2, lowerIsBetter = true } = options;

  if (!Number.isFinite(standardValue) || !Number.isFinite(modifiedValue)) {
    return `${metricName}: could not be compared for this run.`;
  }

  const delta = modifiedValue - standardValue;
  if (Math.abs(delta) < 0.000001) {
    return `${metricName}: both algorithms are effectively equal (${formatNumber(modifiedValue, digits)}${unit}).`;
  }

  const directionWord = delta < 0 ? "lower" : "higher";
  const betterWord = lowerIsBetter ? (delta < 0 ? "better" : "worse") : delta > 0 ? "better" : "worse";
  const deltaPercent = percentDelta(standardValue, modifiedValue);
  const percentText = deltaPercent === null ? "" : ` (${formatSignedNumber(deltaPercent, 1)}%)`;

  return (
    `${metricName}: modified is ${formatNumber(Math.abs(delta), digits)}${unit} ${directionWord}` +
    `${percentText}, so it is ${betterWord} for this metric.`
  );
}

function renderResultExplanation() {
  if (!state.results) {
    elements.resultExplanation.innerHTML =
      "<p class=\"muted-text\">Run both algorithms to generate a case-specific explanation of the differences.</p>";
    return;
  }

  const standard = state.results.standard;
  const modified = state.results.modified;

  if (standard.path.length === 0 || modified.path.length === 0) {
    elements.resultExplanation.innerHTML =
      "<h3>Case-Specific Explanation</h3><p>At least one algorithm could not find a feasible route. In this setup, obstacles or environmental penalties likely disconnected the destination.</p>";
    return;
  }

  const pathChanged = JSON.stringify(standard.path) !== JSON.stringify(modified.path);
  const energyDelta = modified.totalEnergyCost - standard.totalEnergyCost;
  const windDelta = modified.windPenalty - standard.windPenalty;
  const altitudeDelta = modified.altitudePenalty - standard.altitudePenalty;
  const dynamicWindText = state.environment.dynamicWindEnabled
    ? "Dynamic wind is enabled, so the modified solver plans while accounting for the wind shift during traversal."
    : "Wind is static in this run, so differences are caused by directional wind resistance and altitude trade-offs.";

  let interpretation = "In this run, modified Dijkstra improved the targeted energy objective.";
  if (energyDelta > 0.000001) {
    interpretation = "In this run, modified Dijkstra produced a higher evaluated energy cost than standard Dijkstra, which suggests this case is strongly constrained by map geometry or obstacle layout.";
  } else if (Math.abs(energyDelta) <= 0.000001) {
    interpretation = "In this run, both methods produced nearly identical energy cost, indicating the shortest route was also energy-efficient.";
  }

  const windInterpretation =
    windDelta < -0.000001
      ? "Wind exposure is lower for modified Dijkstra, so it avoided headwind-heavy moves."
      : windDelta > 0.000001
        ? "Wind exposure is higher for modified Dijkstra, meaning it traded wind cost for gains elsewhere (such as altitude or route structure)."
        : "Wind exposure is nearly the same for both algorithms in this case.";

  const altitudeInterpretation =
    altitudeDelta < -0.000001
      ? "Altitude penalty is lower for modified Dijkstra, so it selected a smoother elevation profile."
      : altitudeDelta > 0.000001
        ? "Altitude penalty is higher for modified Dijkstra, indicating it accepted more climb/descent to reduce other costs."
        : "Altitude penalty is nearly identical for both algorithms.";

  const pathNarrative = pathChanged
    ? "The two routes differ, which confirms that environmental factors changed path selection beyond pure shortest distance."
    : "The two routes are the same, so shortest-distance and energy-aware optimization aligned in this setup.";

  elements.resultExplanation.innerHTML =
    "<h3>Case-Specific Explanation</h3>" +
    `<p>${pathNarrative}</p>` +
    "<ul>" +
    `<li>${describeMetricDifference("Total energy cost", standard.totalEnergyCost, modified.totalEnergyCost, { lowerIsBetter: true })}</li>` +
    `<li>${describeMetricDifference("Total distance", standard.totalDistance, modified.totalDistance, { lowerIsBetter: true })}</li>` +
    `<li>${describeMetricDifference("Wind penalty", standard.windPenalty, modified.windPenalty, { lowerIsBetter: true })}</li>` +
    `<li>${describeMetricDifference("Altitude penalty", standard.altitudePenalty, modified.altitudePenalty, { lowerIsBetter: true })}</li>` +
    `<li>${describeMetricDifference("Path steps", standard.steps, modified.steps, { lowerIsBetter: true, digits: 0 })}</li>` +
    "</ul>" +
    `<p>${windInterpretation}</p>` +
    `<p>${altitudeInterpretation}</p>` +
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

  const standard = runStandardDijkstra(model);
  const modified = runModifiedDijkstra(model);

  state.results = {
    standard,
    modified,
  };

  renderAll();

  const pathChanged = JSON.stringify(standard.path) !== JSON.stringify(modified.path);

  if (standard.path.length === 0 || modified.path.length === 0) {
    setStatus("No feasible route found. Remove some obstacles or adjust parameters.");
    return;
  }

  setStatus(
    pathChanged
      ? "Algorithms completed. The routes differ under environmental costs."
      : "Algorithms completed. Both selected the same route in this setup.",
  );
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

  setStatus(`Generated ${generated.size} obstacles. Run both algorithms to compare.`);
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
  setStatus("Generated a new altitude map. Run both algorithms to compare routes.");
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
  setStatus(`Loaded ${scenario.name} with ${blockedCount()} obstacles. Click Run Both Algorithms.`);
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
  setStatus(`End selected at ${formatCoord(clickedCoord)}. Click Run Both Algorithms.`);
}

function bindEvents() {
  elements.startTourBtn.addEventListener("click", () => {
    openTour(0);
    setStatus("Guided tour started. Use Next, Back, or Skip.");
  });

  elements.windDirection.addEventListener("change", () => {
    state.environment.windDirection = elements.windDirection.value;
  });

  elements.windStrength.addEventListener("input", () => {
    state.environment.windStrength = Number(elements.windStrength.value);
    elements.windStrengthValue.textContent = state.environment.windStrength.toFixed(1);
  });

  elements.altitudeFactor.addEventListener("input", () => {
    state.environment.altitudeFactor = Number(elements.altitudeFactor.value);
    elements.altitudeFactorValue.textContent = state.environment.altitudeFactor.toFixed(1);
  });

  elements.dynamicWindEnabled.addEventListener("change", () => {
    state.environment.dynamicWindEnabled = elements.dynamicWindEnabled.checked;
    syncWindShiftPanelState();
    state.results = null;
    renderPathSummary();
  });

  elements.windShiftStep.addEventListener("input", () => {
    state.environment.windShiftStep = Number(elements.windShiftStep.value);
    elements.windShiftStepValue.textContent = String(state.environment.windShiftStep);
  });

  elements.windDirectionAfterShift.addEventListener("change", () => {
    state.environment.windDirectionAfterShift = elements.windDirectionAfterShift.value;
  });

  elements.windStrengthAfterShift.addEventListener("input", () => {
    state.environment.windStrengthAfterShift = Number(elements.windStrengthAfterShift.value);
    elements.windStrengthAfterShiftValue.textContent = state.environment.windStrengthAfterShift.toFixed(1);
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

  elements.randomObstacleBtn.addEventListener("click", randomizeObstacles);
  elements.clearObstacleBtn.addEventListener("click", clearObstacles);

  elements.runBtn.addEventListener("click", runAlgorithms);
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
