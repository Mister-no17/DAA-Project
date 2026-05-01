import { performance } from "node:perf_hooks";
import { runModifiedDijkstra, runStandardDijkstra, DEFAULT_ENVIRONMENT } from "./js/algorithms.js";
import { PRESET_SCENARIOS, cloneGrid } from "./js/scenarios.js";

globalThis.performance = performance;

function format(value, digits = 3) {
  if (!Number.isFinite(value)) {
    return "N/A";
  }

  return value.toFixed(digits);
}

function pathsDiffer(firstPath, secondPath) {
  return JSON.stringify(firstPath) !== JSON.stringify(secondPath);
}

for (const scenario of PRESET_SCENARIOS) {
  const obstacleSet = new Set((scenario.obstacles ?? []).map((coord) => `${coord[0]},${coord[1]}`));

  const model = {
    altitudeGrid: cloneGrid(scenario.altitudeGrid),
    start: [...scenario.start],
    end: [...scenario.end],
    environment: {
      ...DEFAULT_ENVIRONMENT,
      ...scenario.environment,
    },
    blockedSet: obstacleSet,
  };

  const standard = runStandardDijkstra(model);
  const modified = runModifiedDijkstra(model);

  console.log("------------------------------------------------------------");
  console.log(`${scenario.name} (${scenario.id})`);
  console.log(`Start: (${scenario.start[0]},${scenario.start[1]})  End: (${scenario.end[0]},${scenario.end[1]})`);
  console.log(`Obstacles: ${obstacleSet.size}`);
  console.log(
    `Wind mode: ${model.environment.dynamicWindEnabled ? `dynamic@step${model.environment.windShiftStep} (${model.environment.windDirection}->${model.environment.windDirectionAfterShift})` : `static (${model.environment.windDirection})`}`,
  );
  console.log(`Paths differ: ${pathsDiffer(standard.path, modified.path) ? "Yes" : "No"}`);
  console.log(
    `Standard  | distance=${format(standard.totalDistance)} energy=${format(standard.totalEnergyCost)} time_ms=${format(
      standard.executionTimeMs,
      6,
    )}`,
  );
  console.log(
    `Modified  | distance=${format(modified.totalDistance)} energy=${format(modified.totalEnergyCost)} time_ms=${format(
      modified.executionTimeMs,
      6,
    )}`,
  );
}

console.log("------------------------------------------------------------");
