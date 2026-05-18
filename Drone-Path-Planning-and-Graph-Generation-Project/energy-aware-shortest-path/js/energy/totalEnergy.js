import { computeDistanceEnergy } from "./distance.js";
import { computeGravityEnergy } from "./gravity.js";
import { computeWindEnergy } from "./wind.js";
import { computeTurningEnergy } from "./turning.js";

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

  const totalEnergy = distanceEnergy + gravityEnergy + windEnergy + turningEnergy;

  return {
    distance,
    altitudeGain,
    turningAngle,
    distanceEnergy,
    gravityEnergy,
    windEnergy,
    turningEnergy,
    totalEnergy,
  };
}
