export function computeTurningEnergy(turningAngleRadians, turningCoefficient) {
  if (!Number.isFinite(turningAngleRadians)) {
    return 0;
  }

  return (turningCoefficient ?? 0) * Math.max(0, turningAngleRadians);
}
