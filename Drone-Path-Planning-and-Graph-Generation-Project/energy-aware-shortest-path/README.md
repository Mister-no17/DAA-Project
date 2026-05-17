# Energy-Aware Shortest Path Algorithm with Environmental Factors

A complete DAA mini-project that compares:

1. Standard Dijkstra (distance-only optimization)
2. Modified Dijkstra (energy-aware optimization)

The modified version uses this custom edge cost:

cost = distance + wind_effect + altitude_cost

where movement direction, wind direction/intensity, and altitude changes influence edge weights.

## Problem Definition

Traditional shortest path minimizes distance, but real systems such as drones often optimize energy. This project models a grid-based environment where each move has an energy implication due to wind and terrain.

Goal:

1. Find a shortest-distance path (baseline).
2. Find a minimum-energy path (modified algorithm).
3. Compare both paths and metrics interactively.

## Project Structure

energy-aware-shortest-path/
- index.html
- styles.css
- js/
  - app.js
  - algorithms.js
  - priorityQueue.js
  - scenarios.js
- scenario_runner.mjs
- TEST_SCENARIOS.md
- DAA_Project_Report.md
- DAA_Report_Print.html
- README.md

## Environment Model

1. Graph model: 2D grid graph.
2. Nodes: grid cells.
3. Edges: 4-neighbor moves (up, down, left, right).
4. Attributes per movement:
- Direction vector (for wind impact)
- Altitude difference (for climb/descent energy)
- Blocked cells (obstacles/no-fly zones)

Additional advanced behavior:

1. Obstacle-aware routing: blocked cells are removed from graph traversal.
2. Sudden wind-shift mode: wind direction/strength can switch after a given step count.
3. Time-expanded Dijkstra is used in dynamic mode so edge cost can depend on traversal time.

### Cost Components

1. Distance:
- Euclidean distance between adjacent cells (equals 1 for 4-neighbor movement).

2. Wind effect:
- wind_effect = wind_strength * (1 - dot(move_unit, wind_unit))
- Tailwind gives lower penalty, headwind gives higher penalty.

3. Altitude cost:
- climb = max(0, altitude_to - altitude_from)
- descent = max(0, altitude_from - altitude_to)
- altitude_cost = altitude_factor * (climb + downhill_factor * descent)

4. Obstacle constraint:
- blocked nodes are treated as non-traversable (effectively infinite cost).

## Algorithm Explanation

### 1) Standard Dijkstra (Baseline)

Standard Dijkstra uses only geometric distance as edge weight.

- Objective: minimize total travel distance
- Weight(u, v) = distance(u, v)

### 2) Modified Dijkstra (Energy-Aware)

The same Dijkstra framework is retained, but edge weight is changed.

- Objective: minimize total energy-aware cost
- Weight(u, v) = distance + wind_effect + altitude_cost

Advanced mode:

1. If dynamic wind is disabled, modified Dijkstra runs on normal node states.
2. If dynamic wind is enabled, modified Dijkstra uses a time-expanded state (node, step).
3. Edge cost is evaluated using wind parameters valid at that specific step.

The key modification is only in edge relaxation weight calculation, making it easy to explain and compare during evaluation.

## Pseudocode

### Standard Dijkstra (Distance)

```text
DIJKSTRA_DISTANCE(grid, start, goal):
    for each node v in grid:
        dist[v] <- INF
        parent[v] <- NIL

    dist[start] <- 0
    pq <- min-priority-queue
    pq.push((0, start))

    while pq is not empty:
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

    return reconstruct_path(parent, start, goal), dist[goal]
```

### Modified Dijkstra (Energy-Aware)

```text
DIJKSTRA_ENERGY(grid, altitude, start, goal, wind_direction, wind_strength, altitude_factor):
    for each node v in grid:
        dist[v] <- INF
        parent[v] <- NIL

    dist[start] <- 0
    pq <- min-priority-queue
    pq.push((0, start))

    while pq is not empty:
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

    return reconstruct_path(parent, start, goal), dist[goal]
```

## Complexity Analysis

Let:

- V = number of grid nodes = rows * cols
- E = number of edges, for 4-neighbor grid E = O(V)

For both algorithms with binary heap priority queue:

1. Time complexity: O((V + E) log V) = O(V log V)
2. Space complexity: O(V) for distance, parent, visited, and queue state

For dynamic wind mode (time-expanded Dijkstra):

1. Let T be the maximum step horizon.
2. Time complexity becomes O((T*V + T*E) log(T*V)).
3. Space complexity becomes O(T*V).

## Trade-offs and Edge Cases

Trade-offs:

1. Standard Dijkstra gives shortest route by distance, but may consume higher energy.
2. Modified Dijkstra may pick a longer route in distance if it significantly reduces environmental cost.
3. Modified version is more realistic for drone planning but requires domain-specific parameter tuning.

Edge cases handled:

1. start == end: zero-step path.
2. Very high wind or altitude factors: still valid because all edge costs remain non-negative.
3. Tied distances: deterministic ordering from priority queue insertion order for reproducible outputs.

## Interactive Features

The interface supports:

1. Selecting start/end points by clicking grid cells.
2. Adjusting wind direction.
3. Adjusting wind strength with slider.
4. Adjusting altitude cost factor with slider.
5. Enabling sudden wind shift (step-triggered direction/strength change).
6. Editing no-fly obstacles directly on the grid.
7. Generating random obstacle fields with density control.
8. Running both algorithms using a button.
9. Visual comparison of both paths in different colors.
10. Displaying total distance, total energy cost, wind penalty, altitude penalty, execution time, expanded nodes, and path steps.

## How to Run

Option A (recommended): local static server

```bash
cd energy-aware-shortest-path
python -m http.server 8000
```

Open:

http://localhost:8000

Option B:

Use VS Code Live Server extension and open index.html.

## Submission Report (PDF Style)

Two report formats are included:

1. DAA_Project_Report.md (editable markdown report)
2. DAA_Report_Print.html (print-optimized report)

To export a PDF report:

1. Open DAA_Report_Print.html in your browser.
2. Press Ctrl+P.
3. Choose Save as PDF.

## Test Scenarios

Three preset scenarios are included in js/scenarios.js:

1. Ridge Escape
2. Mountain Turn
3. Valley Detour

You can run the non-UI scenario runner for quick console output:

```bash
cd energy-aware-shortest-path
node scenario_runner.mjs
```

Measured sample outputs from `node scenario_runner.mjs`:

| Scenario | Obstacles | Wind Mode | Paths Differ | Standard Distance | Standard Energy | Modified Distance | Modified Energy |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Ridge Escape | 6 | static (E) | Yes | 14.000 | 37.740 | 14.000 | 23.700 |
| Mountain Turn | 6 | dynamic step 5 (N -> E) | Yes | 14.000 | 52.300 | 14.000 | 20.000 |
| Valley Detour | 6 | dynamic step 6 (W -> S) | Yes | 14.000 | 53.680 | 14.000 | 17.920 |

See TEST_SCENARIOS.md for full scenario details and expected behavior notes.

## Approach Summary

1. Build a grid graph model with terrain heights.
2. Add no-fly obstacles by removing blocked nodes from feasible transitions.
3. Implement reusable Dijkstra core with pluggable edge weight function.
4. Add time-expanded Dijkstra for dynamic wind-shift conditions.
5. Run baseline and modified versions on identical inputs.
6. Measure and compare route quality and runtime.
7. Visualize both routes for easy interpretation.
