export const PRESET_SCENARIOS = [
  {
    id: "ridge_escape",
    name: "Scenario 1: Ridge Escape",
    description:
      "Steep altitude rise on the left column makes the default shortest path energy-heavy.",
    altitudeGrid: [
      [0, 0, 0, 0, 0, 0, 0, 0],
      [6, 1, 1, 1, 1, 1, 1, 1],
      [7, 1, 1, 1, 1, 1, 1, 1],
      [8, 1, 1, 1, 1, 1, 1, 1],
      [9, 1, 1, 1, 1, 1, 1, 1],
      [9, 1, 1, 1, 1, 1, 1, 1],
      [9, 1, 1, 1, 1, 1, 1, 1],
      [9, 1, 1, 1, 1, 1, 1, 1],
    ],
    start: [0, 0],
    end: [7, 7],
    environment: {
      windDirection: "E",
      windStrength: 1.2,
      altitudeFactor: 1.3,
      dynamicWindEnabled: false,
      windShiftStep: 6,
      windDirectionAfterShift: "W",
      windStrengthAfterShift: 2.2,
    },
    obstacles: [
      [2, 2],
      [3, 2],
      [4, 2],
      [5, 2],
      [5, 3],
      [5, 4],
    ],
  },
  {
    id: "mountain_turn",
    name: "Scenario 2: Mountain Turn",
    description:
      "A high-altitude lower corridor and northward tailwind reward moving upward first.",
    altitudeGrid: [
      [1, 1, 1, 1, 1, 1, 1, 1],
      [1, 2, 2, 2, 2, 2, 2, 1],
      [1, 2, 3, 3, 3, 3, 2, 1],
      [1, 2, 3, 4, 4, 3, 2, 1],
      [1, 2, 3, 4, 5, 3, 2, 1],
      [1, 2, 3, 4, 5, 4, 2, 1],
      [1, 2, 3, 4, 5, 5, 2, 1],
      [0, 7, 8, 9, 9, 9, 9, 1],
    ],
    start: [7, 0],
    end: [0, 7],
    environment: {
      windDirection: "N",
      windStrength: 1.8,
      altitudeFactor: 1.0,
      dynamicWindEnabled: true,
      windShiftStep: 5,
      windDirectionAfterShift: "E",
      windStrengthAfterShift: 2.5,
    },
    obstacles: [
      [6, 2],
      [6, 3],
      [6, 4],
      [5, 4],
      [4, 4],
      [3, 4],
    ],
  },
  {
    id: "valley_detour",
    name: "Scenario 3: Valley Detour",
    description:
      "Descending the right wall is short but costly; moving left early follows a low valley.",
    altitudeGrid: [
      [1, 1, 1, 1, 1, 1, 1, 0],
      [1, 1, 1, 1, 1, 1, 2, 7],
      [1, 1, 1, 1, 1, 2, 2, 8],
      [1, 1, 1, 1, 2, 2, 3, 9],
      [1, 1, 1, 2, 2, 3, 3, 9],
      [1, 1, 2, 2, 3, 3, 4, 9],
      [1, 2, 2, 3, 3, 4, 4, 9],
      [0, 1, 1, 1, 1, 1, 1, 9],
    ],
    start: [0, 7],
    end: [7, 0],
    environment: {
      windDirection: "W",
      windStrength: 1.5,
      altitudeFactor: 1.2,
      dynamicWindEnabled: true,
      windShiftStep: 6,
      windDirectionAfterShift: "S",
      windStrengthAfterShift: 2.3,
    },
    obstacles: [
      [1, 6],
      [2, 6],
      [3, 6],
      [4, 6],
      [5, 6],
      [6, 5],
    ],
  },
];

export function getScenarioById(id) {
  return PRESET_SCENARIOS.find((scenario) => scenario.id === id);
}

export function cloneGrid(grid) {
  return grid.map((row) => row.slice());
}
