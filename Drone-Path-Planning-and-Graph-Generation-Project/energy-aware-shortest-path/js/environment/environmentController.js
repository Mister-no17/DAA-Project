import {
  buildWindTimeline,
  getWindStateAtStep,
  normalizeDynamicEnvironment,
} from "./dynamicWind.js";

export class EnvironmentController {
  constructor(environment = {}) {
    this.reset(environment);
  }

  reset(environment = {}) {
    this.environment = normalizeDynamicEnvironment(environment);
    this.currentStep = 0;
    this.history = [];
  }

  snapshot(step = this.currentStep, from = null, to = null) {
    return getWindStateAtStep(step, this.environment, from, to);
  }

  advanceTo(step) {
    this.currentStep = Math.max(this.currentStep, Math.floor(Number(step) || 0));
    return this.snapshot(this.currentStep);
  }

  scheduleShift(shift) {
    const nextShift = {
      step: Math.max(0, Math.floor(Number(shift?.step) || this.currentStep)),
      direction: shift?.direction ?? this.environment.windDirection,
      strength: Number.isFinite(Number(shift?.strength))
        ? Math.max(0, Number(shift.strength))
        : this.environment.windStrength,
      label: shift?.label ?? "Manual shift",
      source: shift?.source ?? "manual",
    };

    this.environment.windShifts = [...this.environment.windShifts, nextShift].sort(
      (left, right) => left.step - right.step,
    );
    this.environment.dynamicWindEnabled = true;
    return nextShift;
  }

  scheduleGust(gust) {
    const nextGust = {
      startStep: Math.max(0, Math.floor(Number(gust?.startStep) || this.currentStep)),
      endStep: Math.max(
        Math.max(0, Math.floor(Number(gust?.startStep) || this.currentStep)),
        Math.floor(Number(gust?.endStep) || this.currentStep),
      ),
      rows: Array.isArray(gust?.rows) ? gust.rows.slice(0, 2) : [0, 0],
      cols: Array.isArray(gust?.cols) ? gust.cols.slice(0, 2) : [0, 0],
      direction: gust?.direction ?? null,
      strengthMultiplier: Number.isFinite(Number(gust?.strengthMultiplier))
        ? Math.max(0, Number(gust.strengthMultiplier))
        : 1,
      strengthOffset: Number.isFinite(Number(gust?.strengthOffset))
        ? Number(gust.strengthOffset)
        : 0,
      label: gust?.label ?? "Manual gust",
      source: gust?.source ?? "manual",
    };

    this.environment.gustRegions = [...this.environment.gustRegions, nextGust];
    this.environment.dynamicWindEnabled = true;
    return nextGust;
  }

  getTimeline(pathLength) {
    return buildWindTimeline(this.environment, pathLength);
  }

  setBaseWind(direction, strength) {
    this.environment.windDirection = direction ?? this.environment.windDirection;
    this.environment.windStrength = Number.isFinite(Number(strength))
      ? Math.max(0, Number(strength))
      : this.environment.windStrength;
    this.environment.dynamicWindEnabled = Boolean(
      this.environment.dynamicWindEnabled || this.environment.windShifts.length > 0 || this.environment.gustRegions.length > 0,
    );
  }
}

export function createEnvironmentController(environment = {}) {
  return new EnvironmentController(environment);
}
