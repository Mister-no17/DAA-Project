# Test Scenarios and Sample Outputs

This file contains the preset cases used for evaluation.

## Scenario 1: Ridge Escape

1. Start: (0,0)
2. End: (7,7)
3. Environment:
- Wind direction: E
- Wind strength: 1.2
- Altitude factor: 1.3
- Obstacles: 6 blocked nodes
- Dynamic wind shift: Disabled
4. Expected behavior:
- Standard Dijkstra tends to move downward first due deterministic tie order and crosses steep climb.
- Modified Dijkstra chooses a route with lower altitude burden and lower total energy cost.

## Scenario 2: Mountain Turn

1. Start: (7,0)
2. End: (0,7)
3. Environment:
- Wind direction: N
- Wind strength: 1.8
- Altitude factor: 1.0
- Obstacles: 6 blocked nodes
- Dynamic wind shift: Enabled at step 5 (N -> E)
4. Expected behavior:
- Standard shortest path uses distance ties only and becomes expensive when wind shifts.
- Modified Dijkstra uses time-aware energy weighting and adapts path choice under shift conditions.

## Scenario 3: Valley Detour

1. Start: (0,7)
2. End: (7,0)
3. Environment:
- Wind direction: W
- Wind strength: 1.5
- Altitude factor: 1.2
- Obstacles: 6 blocked nodes
- Dynamic wind shift: Enabled at step 6 (W -> S)
4. Expected behavior:
- Standard route may follow a steep wall because it is distance-optimal.
- Modified route can detour through lower valley cells and reduce energy cost after wind direction changes.

## Sample Outputs (Recorded)

Run command used:

```bash
node scenario_runner.mjs
```

### Scenario 1: Ridge Escape

1. Paths differ: Yes
2. Obstacles: 6
3. Wind mode: static (E)
4. Standard: distance=14.000, energy=37.740
5. Modified: distance=14.000, energy=23.700

### Scenario 2: Mountain Turn

1. Paths differ: Yes
2. Obstacles: 6
3. Wind mode: dynamic step 5 (N -> E)
4. Standard: distance=14.000, energy=52.300
5. Modified: distance=14.000, energy=20.000

### Scenario 3: Valley Detour

1. Paths differ: Yes
2. Obstacles: 6
3. Wind mode: dynamic step 6 (W -> S)
4. Standard: distance=14.000, energy=53.680
5. Modified: distance=14.000, energy=17.920

These results show the same-distance route can have very different energy costs, and obstacle + time-dependent wind modeling creates a much stronger distinction between baseline and modified behavior.
