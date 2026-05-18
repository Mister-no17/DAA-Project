export function computeTurningEnergy(turningAngleRadians, turningCoefficient) {
  if (!Number.isFinite(turningAngleRadians)) {
    return 0;
  }

  const angle = Math.max(0, turningAngleRadians);
  // Quadratic penalty models curvature/yaw effort: sharper turns cost disproportionately more.
  return (turningCoefficient ?? 0) * (angle * angle);
}
