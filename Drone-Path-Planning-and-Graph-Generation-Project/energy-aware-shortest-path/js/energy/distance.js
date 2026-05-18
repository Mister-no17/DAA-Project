export function computeDistanceEnergy(distance, distanceCoefficient) {
  if (!Number.isFinite(distance)) {
    return Number.POSITIVE_INFINITY;
  }

  return (distanceCoefficient ?? 1) * distance;
}
