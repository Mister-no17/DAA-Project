export function computeGravityEnergy(altitudeGain, mass, gravity) {
  if (!Number.isFinite(altitudeGain)) {
    return Number.POSITIVE_INFINITY;
  }

  const climb = Math.max(0, altitudeGain);
  return (mass ?? 1) * (gravity ?? 9.81) * climb;
}
