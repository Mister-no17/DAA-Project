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

const DEFAULT_DYNAMIC_ENVIRONMENT = Object.freeze({
  windDirection: "E",
  windStrength: 1.5,
  dynamicWindEnabled: false,
  windShiftStep: 6,
  windDirectionAfterShift: "W",
  windStrengthAfterShift: 2.2,
  maxStepMultiplier: 2.5,
  windShifts: [],
  gustRegions: [],
});

function toFiniteNumber(value, fallback = 0) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function normalizeDirection(direction, fallback = DEFAULT_DYNAMIC_ENVIRONMENT.windDirection) {
  return Object.prototype.hasOwnProperty.call(DIRECTION_VECTORS, direction) ? direction : fallback;
}

function normalizeRange(range, fallbackMin = 0, fallbackMax = 0) {
  if (Array.isArray(range) && range.length >= 2) {
    const min = Math.min(toFiniteNumber(range[0], fallbackMin), toFiniteNumber(range[1], fallbackMax));
    const max = Math.max(toFiniteNumber(range[0], fallbackMin), toFiniteNumber(range[1], fallbackMax));
    return [min, max];
  }

  return [fallbackMin, fallbackMax];
}

function normalizeShiftEvent(event, fallbackStep, fallbackDirection, fallbackStrength, index = 0) {
  const step = Math.max(0, Math.floor(toFiniteNumber(event?.step, fallbackStep)));
  const direction = normalizeDirection(event?.direction, fallbackDirection);
  const strength = Math.max(0, toFiniteNumber(event?.strength, fallbackStrength));
  const transitionSteps = Math.max(0, Math.floor(toFiniteNumber(event?.transitionSteps, 0)));

  return {
    step,
    direction,
    strength,
    transitionSteps,
    label: event?.label ?? `Shift ${index + 1}`,
    source: event?.source ?? "scenario",
  };
}

function normalizeGustRegion(region, index = 0) {
  const startStep = Math.max(0, Math.floor(toFiniteNumber(region?.startStep, 0)));
  const endStep = Math.max(startStep, Math.floor(toFiniteNumber(region?.endStep, startStep)));
  const rows = normalizeRange(region?.rows, 0, 0);
  const cols = normalizeRange(region?.cols, 0, 0);

  return {
    startStep,
    endStep,
    rows,
    cols,
    direction: region?.direction ? normalizeDirection(region.direction, null) : null,
    strengthMultiplier: Math.max(0, toFiniteNumber(region?.strengthMultiplier, 1)),
    strengthOffset: toFiniteNumber(region?.strengthOffset, 0),
    label: region?.label ?? `Gust ${index + 1}`,
    source: region?.source ?? "scenario",
  };
}

function normalizeCellKey(coord) {
  if (!Array.isArray(coord) || coord.length < 2) {
    return null;
  }

  const row = Number(coord[0]);
  const col = Number(coord[1]);

  if (!Number.isFinite(row) || !Number.isFinite(col)) {
    return null;
  }

  return [Math.floor(row), Math.floor(col)];
}

function isWithinRegion(coord, region) {
  const normalized = normalizeCellKey(coord);
  if (!normalized) {
    return false;
  }

  const [row, col] = normalized;
  const [minRow, maxRow] = region.rows;
  const [minCol, maxCol] = region.cols;

  return row >= minRow && row <= maxRow && col >= minCol && col <= maxCol;
}

function collectWindShiftEvents(environment) {
  const shifts = Array.isArray(environment?.windShifts) ? environment.windShifts : [];

  if (shifts.length > 0) {
    return shifts
      .map((event, index) =>
        normalizeShiftEvent(
          event,
          environment?.windShiftStep ?? DEFAULT_DYNAMIC_ENVIRONMENT.windShiftStep,
          environment?.windDirectionAfterShift ?? environment?.windDirection,
          environment?.windStrengthAfterShift ?? environment?.windStrength,
          index,
        ),
      )
      .sort((left, right) => left.step - right.step);
  }

  if (!environment?.dynamicWindEnabled) {
    return [];
  }

  return [
    normalizeShiftEvent(
      {
        step: environment?.windShiftStep ?? DEFAULT_DYNAMIC_ENVIRONMENT.windShiftStep,
        direction: environment?.windDirectionAfterShift ?? environment?.windDirection,
        strength: environment?.windStrengthAfterShift ?? environment?.windStrength,
        label: "Legacy wind shift",
        source: "legacy",
      },
      environment?.windShiftStep ?? DEFAULT_DYNAMIC_ENVIRONMENT.windShiftStep,
      environment?.windDirectionAfterShift ?? environment?.windDirection,
      environment?.windStrengthAfterShift ?? environment?.windStrength,
      0,
    ),
  ];
}

function collectGustRegions(environment) {
  const gusts = Array.isArray(environment?.gustRegions)
    ? environment.gustRegions
    : Array.isArray(environment?.windGusts)
      ? environment.windGusts
      : [];

  return gusts.map((region, index) => normalizeGustRegion(region, index));
}

export function normalizeDynamicEnvironment(environment = {}) {
  const merged = {
    ...DEFAULT_DYNAMIC_ENVIRONMENT,
    ...environment,
  };

  const shifts = collectWindShiftEvents(merged);
  const gustRegions = collectGustRegions(merged);

  return {
    ...merged,
    windDirection: normalizeDirection(merged.windDirection),
    windStrength: Math.max(0, toFiniteNumber(merged.windStrength, DEFAULT_DYNAMIC_ENVIRONMENT.windStrength)),
    windShiftStep: Math.max(0, Math.floor(toFiniteNumber(merged.windShiftStep, DEFAULT_DYNAMIC_ENVIRONMENT.windShiftStep))),
    windDirectionAfterShift: normalizeDirection(
      merged.windDirectionAfterShift,
      normalizeDirection(merged.windDirection),
    ),
    windStrengthAfterShift: Math.max(
      0,
      toFiniteNumber(merged.windStrengthAfterShift, DEFAULT_DYNAMIC_ENVIRONMENT.windStrengthAfterShift),
    ),
    maxStepMultiplier: Math.max(1, toFiniteNumber(merged.maxStepMultiplier, DEFAULT_DYNAMIC_ENVIRONMENT.maxStepMultiplier)),
    dynamicWindEnabled: Boolean(merged.dynamicWindEnabled || shifts.length > 0 || gustRegions.length > 0),
    windShifts: shifts,
    gustRegions,
  };
}

function resolveWindStateFromGusts(step, from, to, environment, currentState) {
  let state = { ...currentState };

  for (const gust of environment.gustRegions) {
    const gustActive = step >= gust.startStep && step <= gust.endStep;
    if (!gustActive) {
      continue;
    }

    if (!isWithinRegion(from, gust) && !isWithinRegion(to, gust)) {
      continue;
    }

    if (gust.direction) {
      state.direction = gust.direction;
    }

    state.strength = Math.max(0, state.strength * gust.strengthMultiplier + gust.strengthOffset);
    state.label = gust.label;
    state.hasGust = true;
  }

  return state;
}

export function getWindStateAtStep(step, environment, from = null, to = null) {
  const normalized = normalizeDynamicEnvironment(environment);
  const absoluteStep = Math.max(0, Math.floor(toFiniteNumber(step, 0)));

  let activeState = {
    direction: normalized.windDirection,
    strength: normalized.windStrength,
    label: "Base wind",
    hasGust: false,
    shiftIndex: -1,
  };

  for (let index = 0; index < normalized.windShifts.length; index += 1) {
    const shift = normalized.windShifts[index];
    const transitionStart = Math.max(0, shift.step - (shift.transitionSteps ?? 0));
    const transitionSteps = shift.transitionSteps ?? 0;

    if (transitionSteps > 0 && absoluteStep >= transitionStart && absoluteStep < shift.step) {
      const progress = (absoluteStep - transitionStart + 1) / transitionSteps;
      const clamped = Math.max(0, Math.min(1, progress));
      const previousStrength = activeState.strength;
      const blendedStrength = previousStrength + (shift.strength - previousStrength) * clamped;

      activeState = {
        ...activeState,
        strength: blendedStrength,
        label: shift.label,
        shiftIndex: index,
        lastShiftStep: shift.step,
        transitionStep: absoluteStep,
      };

      continue;
    }

    if (absoluteStep < shift.step) {
      break;
    }

    activeState = {
      ...activeState,
      direction: shift.direction,
      strength: shift.strength,
      label: shift.label,
      shiftIndex: index,
      lastShiftStep: shift.step,
    };
  }

  return resolveWindStateFromGusts(absoluteStep, from, to, normalized, activeState);
}

export function getWindShiftEvents(environment) {
  return normalizeDynamicEnvironment(environment).windShifts.slice();
}

export function getWindChangeSteps(environment) {
  const normalized = normalizeDynamicEnvironment(environment);
  const steps = new Set(normalized.windShifts.map((shift) => shift.step));

  normalized.windShifts.forEach((shift) => {
    if (shift.transitionSteps && shift.transitionSteps > 0) {
      const transitionStart = Math.max(0, shift.step - shift.transitionSteps);
      steps.add(transitionStart);
    }
  });

  normalized.gustRegions.forEach((gust) => {
    steps.add(gust.startStep);
    steps.add(gust.endStep);
  });

  return Array.from(steps).sort((left, right) => left - right);
}

export function buildWindTimeline(environment, pathLength) {
  const normalized = normalizeDynamicEnvironment(environment);
  const horizon = Math.max(0, Math.floor(toFiniteNumber(pathLength, 0)));
  const states = [];
  let shiftCount = 0;
  let gustCount = 0;
  let previousSignature = null;

  for (let step = 0; step <= horizon; step += 1) {
    const state = getWindStateAtStep(step, normalized);
    const signature = `${state.direction}|${state.strength.toFixed(3)}|${state.hasGust ? 1 : 0}`;

    if (previousSignature !== null && signature !== previousSignature) {
      shiftCount += 1;
    }

    if (state.hasGust) {
      gustCount += 1;
    }

    states.push({
      step,
      direction: state.direction,
      strength: state.strength,
      label: state.label,
      hasGust: state.hasGust,
      shiftIndex: state.shiftIndex,
      lastShiftStep: state.lastShiftStep ?? null,
    });

    previousSignature = signature;
  }

  return {
    states,
    shiftCount,
    gustCount,
    eventCount: normalized.windShifts.length + normalized.gustRegions.length,
  };
}

export function describeWindState(state) {
  if (!state) {
    return "Wind unavailable";
  }

  const gustLabel = state.hasGust ? " gust" : "";
  return `${state.direction} ${state.strength.toFixed(1)}${gustLabel}`.trim();
}

export function createWindEvolutionSummary(environment, pathLength) {
  const timeline = buildWindTimeline(environment, pathLength);

  return {
    ...timeline,
    activeShifts: getWindShiftEvents(environment),
    changeSteps: getWindChangeSteps(environment),
  };
}
