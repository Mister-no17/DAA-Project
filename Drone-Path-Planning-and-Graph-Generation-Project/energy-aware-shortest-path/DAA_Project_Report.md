# Energy-Aware Shortest Path Algorithm with Environmental Factors

## DAA Course Mini-Project Report

- Student Name: ____________________
- Roll Number: _____________________
- Course: Design and Analysis of Algorithms
- Submission Date: __________________

## 1. Problem Definition

Traditional shortest path algorithms minimize distance only. In real navigation problems (for example, drones), minimum distance is not always minimum energy. Environmental conditions such as wind and terrain elevation can increase or reduce movement cost.

This project compares:

1. Standard Dijkstra's Algorithm (distance optimized)
2. Modified Dijkstra's Algorithm (energy optimized)

The modified edge cost is:

cost = distance + wind_effect + altitude_cost

## 2. Objective

1. Build a 2D grid-based graph.
2. Implement baseline and modified Dijkstra algorithms.
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

wind_effect = wind_strength * (1 - dot(move_unit, wind_unit))

### 3.3 Altitude Model

Each cell contains an altitude value.

1. Climbing consumes more energy.
2. Descending has a smaller cost (scaled by downhill_factor).

Formula used:

- climb = max(0, altitude_to - altitude_from)
- descent = max(0, altitude_from - altitude_to)
- altitude_cost = altitude_factor * (climb + downhill_factor * descent)

### 3.4 Sudden Wind Shift Model

The system supports abrupt wind changes after a selected step index.

1. For step < shift_step: use initial wind direction/strength.
2. For step >= shift_step: use post-shift wind direction/strength.

This makes edge cost time-dependent and requires time-expanded shortest path search.

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

### 4.2 Modified Dijkstra (Energy-Aware)

- Objective: Minimize energy-aware cost.
- Edge Weight: distance + wind_effect + altitude_cost

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

### 4.3 Time-Expanded Modified Dijkstra (Dynamic Wind)

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

## 5. Key Modification Explanation

The algorithmic framework remains Dijkstra. The only change is the edge cost function used during relaxation.

- Baseline: w = distance
- Modified: w = distance + wind_effect + altitude_cost

This isolates the effect of environmental modeling while preserving correctness and complexity characteristics of Dijkstra under non-negative weights.

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
2. Modified Dijkstra is more realistic but depends on parameter tuning.
3. Energy-optimal path can differ from distance-optimal path.
4. Dynamic wind mode is more realistic but increases runtime and memory due time-expanded states.

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
7. Button to run both algorithms.
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
