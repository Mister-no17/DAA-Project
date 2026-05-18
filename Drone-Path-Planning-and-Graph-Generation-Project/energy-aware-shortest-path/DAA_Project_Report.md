# Energy-Aware Minimum-Energy Trajectory Optimization

## DAA Course Mini-Project Report

- Student Name: ____________________
- Roll Number: _____________________
- Course: Design and Analysis of Algorithms
- Submission Date: __________________

## 1. Problem Definition

Traditional distance-minimizing routing optimizes path length only. In real navigation problems (for example, drones), minimum distance is not always minimum energy. Environmental conditions such as wind and terrain elevation can increase or reduce movement cost.

This project compares:

1. Standard Dijkstra's Algorithm (distance optimized)
2. Energy-Aware A* (energy optimized)

The energy-aware edge cost is:

E_total = E_distance + E_gravity + E_wind + E_turn

## 2. Objective

1. Build a 2D grid-based graph.
2. Implement baseline and energy-aware routing algorithms.
3. Add obstacle-aware routing (no-fly zones).
4. Add time-dependent wind shift handling.
5. Compare both algorithms under controllable environmental parameters.
4. Visualize and analyze path differences, runtime, and cost metrics.

## 3. Environment Model

### 3.1 Graph Representation

1. Grid cells are nodes.
2. Each node connects to four neighbors: up, down, left, right.
3. Each movement has geometric distance = 1 (for 4-neighbor movement).
4. Blocked cells are treated as obstacles and removed from feasible transitions.

### 3.2 Wind Model

1. Wind direction is selected from {N, NE, E, SE, S, SW, W, NW}.
2. Wind strength is controlled by a slider.
3. Movement aligned with wind has lower penalty than movement against wind.

Formula used:

E_wind = k_w * wind_strength * (1 - cos(theta))

### 3.3 Gravity Model

Each cell contains an altitude value.

1. Climbing consumes more energy.
2. Descending adds no gravity cost.

Formula used:

- altitude_gain = max(0, altitude_to - altitude_from)
- E_gravity = m * g * altitude_gain

### 3.4 Sudden Wind Shift Model

The system supports abrupt wind changes after a selected step index.

1. For step < shift_step: use initial wind direction/strength.
2. For step >= shift_step: use post-shift wind direction/strength.

This makes edge cost time-dependent and requires time-expanded minimum-energy search.

### 3.5 Multi-Phase Wind and Gust Regions

Wind is represented as a sequence of phases (wind shifts) and optional localized gust regions. Each phase can change
direction and strength, and gusts temporarily amplify wind inside specified grid regions. This supports storm fronts,
wind corridors, and directional reversals.

### 3.6 Turning/Yaw Model

Turning cost is modeled as a curvature penalty:

E_turn = k_t * theta^2

This penalizes sharp heading changes more than smooth turns, approximating UAV yaw effort and stabilization energy.

## 4. Algorithms

### 4.1 Standard Dijkstra (Baseline)

- Objective: Minimize total distance.
- Edge Weight: distance(u, v)

Pseudocode:

```text
DIJKSTRA_DISTANCE(grid, start, goal):
    for each node v:
        dist[v] <- INF
        parent[v] <- NIL

    dist[start] <- 0
    pq.push((0, start))

    while pq not empty:
        (d, u) <- pq.pop_min()
        if d > dist[u]:
            continue
        if u == goal:
            break

        for each neighbor v of u:
            w <- geometric_distance(u, v)
            if dist[u] + w < dist[v]:
                dist[v] <- dist[u] + w
                parent[v] <- u
                pq.push((dist[v], v))

    return reconstruct_path(parent), dist[goal]
```

### 4.2 Energy-Aware A* (Energy-Aware)

- Objective: Minimize energy-aware cost.
- Edge Weight: distance + wind_effect + altitude_cost
- Heuristic: admissible lower bound on remaining energy

Pseudocode:

```text
DIJKSTRA_ENERGY(grid, altitude, start, goal, wind_direction, wind_strength, altitude_factor):
    for each node v:
        dist[v] <- INF
        parent[v] <- NIL

    dist[start] <- 0
    pq.push((0, start))

    while pq not empty:
        (d, u) <- pq.pop_min()
        if d > dist[u]:
            continue
        if u == goal:
            break

        for each neighbor v of u:
            distance <- geometric_distance(u, v)
            wind_effect <- wind_strength * (1 - dot(move_unit(u, v), wind_unit(wind_direction)))
            climb <- max(0, altitude[v] - altitude[u])
            descent <- max(0, altitude[u] - altitude[v])
            altitude_cost <- altitude_factor * (climb + downhill_factor * descent)

            w <- distance + wind_effect + altitude_cost

            if dist[u] + w < dist[v]:
                dist[v] <- dist[u] + w
                parent[v] <- u
                pq.push((dist[v], v))

    return reconstruct_path(parent), dist[goal]
```

### 4.3 Time-Expanded Energy-Aware A* (Dynamic Wind)

When dynamic wind is enabled, state becomes (node, step) instead of only node.

```text
DIJKSTRA_ENERGY_TIME(grid, altitude, start, goal, shift_step, wind_before, wind_after):
    initialize dist[(v, t)] = INF for t in [0..T]
    dist[(start, 0)] <- 0
    pq.push((0, start, 0))

    while pq not empty:
        (d, u, t) <- pq.pop_min()
        if d > dist[(u, t)]:
            continue
        if u == goal:
            return path, d

        for each neighbor v of u not blocked:
            wind <- wind_before if t < shift_step else wind_after
            w <- distance + wind_effect(u, v, wind) + altitude_cost(u, v)

            if dist[(u, t)] + w < dist[(v, t+1)]:
                dist[(v, t+1)] <- dist[(u, t)] + w
                parent[(v, t+1)] <- (u, t)
                pq.push((dist[(v, t+1)], v, t+1))

    return no_path
```

### 4.4 Adaptive Replanning (Dynamic Wind)

When wind changes beyond thresholds, the planner recomputes a new route from the current state. If the wind change is
minor, the residual plan is reused to avoid unnecessary recomputation. This approximates incremental replanning without
full algorithmic rewrites while preserving correctness for significant shifts.

### 4.5 Heuristic Admissibility (Energy-Aware A*)

We use a lower-bound heuristic:

h(n) = k_d * dist(n, goal) + m * g * max(0, h(goal) - h(n))

Wind and turning terms are lower-bounded by 0 because the model assigns non-negative energy. This ensures h(n)
never overestimates the true remaining energy, preserving A* optimality under non-negative edge costs.

If optional per-step lower bounds are supplied, they must be guaranteed minima to preserve admissibility.

### 4.6 Consistency (Monotonicity)

Consistency requires:

h(n) <= c(n, n') + h(n')

With non-negative costs and a straight-line distance lower bound, the heuristic is typically consistent. It can be
violated if parameters make the heuristic exceed the minimal per-step energy (for example, overly aggressive scaling).

## 5. Key Modification Explanation

The algorithmic framework remains Dijkstra. The only change is the edge cost function used during relaxation.

- Baseline: w = distance
- Modified: w = distance + wind_effect + altitude_cost

This isolates the effect of environmental modeling while preserving correctness and complexity characteristics of Dijkstra under non-negative weights.

## 5.1 Theta* Energy Reduction Rationale

Theta* permits any-angle paths via LOS shortcuts, which reduces zig-zag motion. Under a quadratic turning penalty
(E_turn = k_t * theta^2), smoother paths reduce cumulative yaw energy and stabilization effort, yielding lower total energy
compared to grid-constrained routes.

## 5.2 Dynamic Routing Rationale

Static shortest paths are brittle under changing wind. By treating cost as a function of time, the planner adapts to
environmental shifts and avoids regions that become expensive later in the route. This is critical for autonomous UAV
navigation in non-stationary conditions.

## 6. Complexity Analysis

Let V be number of nodes and E be number of edges.

For a rows x cols grid:

- V = rows * cols
- E = O(V) for 4-neighbor adjacency

Using a binary heap priority queue:

1. Time Complexity: O((V + E) log V) = O(V log V)
2. Space Complexity: O(V)

Both baseline and modified versions have the same asymptotic complexity.

For dynamic wind mode with time-expanded states:

1. Let T be max step horizon.
2. Time Complexity: O((T*V + T*E) log(T*V))
3. Space Complexity: O(T*V)

Energy-Aware Theta* adds LOS-based shortcutting and dynamic parent relaxation. It is still fundamentally an A* search, but it evaluates additional straight-line visibility segments before committing a successor edge.

- Recurrence relation: No standard divide-and-conquer recurrence. Theta* extends A* using line-of-sight relaxation and any-angle parent propagation.
- Recurrence applicability: Theta* combines heuristic graph search with LOS traversal and dynamic parent relaxation for smoother any-angle routing. Heuristic guidance reduces expansion, LOS shortcuts shorten the effective search tree, and parent propagation smooths heading changes while preserving feasibility.

Time Complexity:
1. Best Case: O((V + E) log V) — heuristic guidance dominates and few LOS checks are needed.
2. Average/Worst Case: O((V + E) log V + L_total) or equivalently O((V + E) log V + LOS_checks).
   - LOS traversal overhead arises from Bresenham line checks across the grid.
   - Each LOS check costs O(length_of_segment) in the number of traversed cells.
   - Obstacle density directly affects LOS evaluation: dense obstacles increase failed LOS attempts and force more local expansions.
   - The algorithm’s runtime depends on both graph operations and the visibility-check budget.

Here L_total is the total Bresenham LOS traversal length across all checks.

Adaptive replanning adds additional A*/Theta* runs, but only at wind change points that exceed thresholds. In practice, this bounds recomputation to the number of significant environment shifts rather than every step.

## 7. Experimental Scenarios and Results

### 7.1 Preset Scenarios

1. Ridge Escape
2. Mountain Turn
3. Valley Detour

### 7.2 Recorded Outputs

| Scenario | Obstacles | Wind Mode | Paths Differ | Standard Distance | Standard Energy | Modified Distance | Modified Energy |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Ridge Escape | 6 | static (E) | Yes | 14.000 | 37.740 | 14.000 | 23.700 |
| Mountain Turn | 6 | dynamic step 5 (N -> E) | Yes | 14.000 | 52.300 | 14.000 | 20.000 |
| Valley Detour | 6 | dynamic step 6 (W -> S) | Yes | 14.000 | 53.680 | 14.000 | 17.920 |

Observation:

In all three scenarios, path selection changed under environmental costs and the modified algorithm achieved lower total energy cost.

## 8. Trade-offs and Edge Cases

### Trade-offs

1. Standard Dijkstra is simpler but ignores physical realism.
2. Energy-Aware A* is more realistic but depends on parameter tuning.
3. Energy-optimal path can differ from distance-optimal path.
4. Dynamic wind mode is more realistic but increases runtime and memory due time-expanded states.

### Correctness and Assumptions

1. Edge costs are non-negative (distance, gravity, wind, turning).
2. Admissible heuristic preserves A* optimality when costs are non-negative.
3. Theta* optimality is relative to the discrete LOS graph induced by Bresenham traversal.
4. LOS assumes straight-line traversal through grid cells is feasible if all cells on the line are unblocked.
5. Adaptive replanning reuses residual paths when changes are minor; this is an approximation but preserves safety if
    replan thresholds are conservative.

### Edge Cases

1. Start equals end.
2. High wind and altitude factors.
3. Ties in path cost (handled deterministically by queue order).
4. Start or end blocked by obstacles (returns no feasible path).
5. Dense obstacles disconnecting the graph.

## 9. Interactive Interface Features

The UI provides:

1. Start/end node selection on the grid.
2. Wind direction selector.
3. Wind strength slider.
4. Altitude cost slider.
5. Sudden wind shift toggle and shift-step controls.
6. Obstacle paint mode and random obstacle generation.
7. Button to run all algorithms.
8. Color-coded path visualization and comparison table.

## 10. Conclusion

The project demonstrates a clear and practical Dijkstra modification for energy-aware routing. Results show that shortest distance is not always the best route under environmental constraints. The modified algorithm preserves Dijkstra's computational efficiency while producing more realistic path planning decisions.

## 11. Reproducibility

Run UI:

```bash
python -m http.server 8000
```

Then open http://localhost:8000 from the project folder.

Run scenario report in console:

```bash
node scenario_runner.mjs
```
