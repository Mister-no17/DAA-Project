export function computeWindEnergy(from, to, windDirection, windStrength, windCoefficient, directionVectors) {
  const windVector = directionVectors?.[windDirection] ?? directionVectors?.E;
  if (!windVector) {
    return 0;
  }

  const moveRow = to[0] - from[0];
  const moveCol = to[1] - from[1];
  const moveNorm = Math.hypot(moveRow, moveCol) || 1;
  const windNorm = Math.hypot(windVector[0], windVector[1]) || 1;

  const unitMoveRow = moveRow / moveNorm;
  const unitMoveCol = moveCol / moveNorm;
  const unitWindRow = windVector[0] / windNorm;
  const unitWindCol = windVector[1] / windNorm;

  const dot = unitMoveRow * unitWindRow + unitMoveCol * unitWindCol;
  const clampedDot = Math.max(-1, Math.min(1, dot));
  const theta = Math.acos(clampedDot);

  return (windCoefficient ?? 1) * (windStrength ?? 0) * (1 - Math.cos(theta));
}
