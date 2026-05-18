import { computeDistanceEnergy } from "./distance.js";
import { computeGravityEnergy } from "./gravity.js";
import { computeWindEnergy } from "./wind.js";
import { computeTurningEnergy } from "./turning.js";

/**
 * computeNormalizationBounds
 * Returns safe, scenario-aware upper bounds for per-edge components used to
 * normalize heterogeneous energy terms. Bounds are intentionally conservative
 * (upper estimates) to ensure stability across scenarios and avoid divide-by-zero.
 */
export function computeNormalizationBounds(altitudeGrid, environment) {
  const rows = altitudeGrid?.length || 1;
  const cols = altitudeGrid?.[0]?.length || 1;

  // Distance: maximal straight-line distance across the grid (upper bound for an edge).
  const D_max = Math.hypot(Math.max(0, rows - 1), Math.max(0, cols - 1)) || 1;

  // Gravity: maximal positive altitude gain possible between any two cells (upper bound).
  let maxAlt = -Infinity;
  let minAlt = Infinity;
  for (let r = 0; r < rows; r += 1) {
    for (let c = 0; c < (altitudeGrid[r] || []).length; c += 1) {
      const v = altitudeGrid[r][c];
      if (Number.isFinite(v)) {
        maxAlt = Math.max(maxAlt, v);
        minAlt = Math.min(minAlt, v);
      }
    }
  }
  if (!Number.isFinite(maxAlt)) {
    maxAlt = 0;
  }
  if (!Number.isFinite(minAlt)) {
    minAlt = 0;
  }
  const maxAltitudeGain = Math.max(0, maxAlt - minAlt);
  const G_max = (environment?.mass ?? 1) * (environment?.gravity ?? 9.81) * (maxAltitudeGain || 1);

  // Wind: worst-case angular penalty is (1 - cos(pi)) = 2. Use maximal configured wind strength.
  const windStrengthMax = Math.max(environment?.windStrength ?? 0, environment?.windStrengthAfterShift ?? 0, 1);
  const W_max = (environment?.windCoefficient ?? 1) * windStrengthMax * 2;

  // Turning: maximal per-edge turning is pi radians, quadratic penalty => pi^2.
  const T_max = (environment?.turningCoefficient ?? 1) * (Math.PI * Math.PI || 1);

  // Ensure no zeros to avoid divide-by-zero in normalization.
  return {
    D_max: D_max || 1,
    G_max: G_max || 1,
    W_max: W_max || 1,
    T_max: T_max || 1,
  };
}

function vectorBetween(from, to) {
  return [to[0] - from[0], to[1] - from[1]];
}

function turningAngleRadians(previousVector, nextVector) {
  if (!previousVector || !nextVector) {
    return 0;
  }

  const [pr, pc] = previousVector;
  const [nr, nc] = nextVector;
  const prevNorm = Math.hypot(pr, pc);
  const nextNorm = Math.hypot(nr, nc);

  if (prevNorm === 0 || nextNorm === 0) {
    return 0;
  }

  const dot = (pr * nr + pc * nc) / (prevNorm * nextNorm);
  const clampedDot = Math.max(-1, Math.min(1, dot));
  return Math.acos(clampedDot);
}

export function computeEnergyComponents({
  from,
  to,
  prev,
  altitudeGrid,
  environment,
  windState,
  directionVectors,
}) {
  const distance = Math.hypot(to[0] - from[0], to[1] - from[1]);
  const altitudeFrom = altitudeGrid[from[0]][from[1]];
  const altitudeTo = altitudeGrid[to[0]][to[1]];
  const altitudeGain = altitudeTo - altitudeFrom;

  const distanceEnergy = computeDistanceEnergy(distance, environment.distanceCoefficient);
  const gravityEnergy = computeGravityEnergy(altitudeGain, environment.mass, environment.gravity);
  const windEnergy = computeWindEnergy(
    from,
    to,
    windState.direction,
    windState.strength,
    environment.windCoefficient,
    directionVectors,
  );

  const previousVector = prev ? vectorBetween(prev, from) : null;
  const currentVector = vectorBetween(from, to);
  const turningAngle = turningAngleRadians(previousVector, currentVector);
  const turningEnergy = computeTurningEnergy(turningAngle, environment.turningCoefficient);

  // Physical energy total is the raw sum of component energies.
  const totalPhysicalEnergy = distanceEnergy + gravityEnergy + windEnergy + turningEnergy;

  // --- Normalization and weighted aggregation ---
  // Compute conservative, scenario-aware bounds for normalizing heterogeneous terms.
  const bounds = computeNormalizationBounds(altitudeGrid, environment);

  const D_norm = distance / bounds.D_max;
  const G_norm = Math.max(0, gravityEnergy) / bounds.G_max;
  const W_norm = Math.max(0, windEnergy) / bounds.W_max;
  const T_norm = Math.max(0, turningEnergy) / bounds.T_max;

  // Allow per-environment weights to tune relative importance; fall back to defaults.
  const weights = (environment && environment.weights) || { wd: 0.25, wg: 0.35, ww: 0.25, wt: 0.15 };

  const totalNormalized = (weights.wd ?? 0) * D_norm + (weights.wg ?? 0) * G_norm + (weights.ww ?? 0) * W_norm + (weights.wt ?? 0) * T_norm;

  return {
    distance,
    altitudeGain,
    turningAngle,
    // raw component energies (physical-like units)
    distanceEnergy,
    gravityEnergy,
    windEnergy,
    turningEnergy,
    totalPhysicalEnergy,
    // normalized components (unitless, in [0, ~1] w.r.t conservative upper bounds)
    distanceEnergyNormalized: D_norm,
    gravityEnergyNormalized: G_norm,
    windEnergyNormalized: W_norm,
    turningEnergyNormalized: T_norm,
    // weights used for aggregation
    weights,
    // final objective value used during search (normalized weighted units)
    totalEnergy: totalNormalized,
  };
}
