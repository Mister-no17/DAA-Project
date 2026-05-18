# Complexity Analysis — Energy-Aware Shortest Path Project

This document explains time and space complexity for the core parts of the project and gives simple recurrence relations that capture growth.

Definitions
- r = number of rows, c = number of cols
- V = r × c (number of grid nodes)
- E ≈ 4V (4-neighbor grid, constant-degree)
- For time-expanded search (dynamic wind) let H = horizon (max steps) and N = V × H (number of time-expanded states)

Notation: O(·) hides constant factors; log means logarithm base 2.

---

1) Cost of primitive operations
- Priority queue `push` / `pop`: O(log M) where M is current queue size.
- Edge weight computation (`computeEnergyEdgeCost` / `computeEnergyEdgeCostAtStep`): O(1) (few arithmetic ops and a dot product)
- Neighbor iteration for a node: O(1) (at most 4 neighbors)

These determine the per-relaxation cost used in Dijkstra.

---

2) Standard Dijkstra (one-sided) — single-state grid (`dijkstraGrid`)

Work model (worst-case): every node may be settled once and each edge considered once (constant-degree). Each relaxation may push a new entry into the heap.

Time:
- For V nodes and E edges: O((V + E) log V) = O(V log V) (because E = Θ(V) here).
- More detailed: each pop is O(log V) and there are up to V pops; each push is O(log V) and there are up to O(E) pushes; result O((V + E) log V).

Space:
- `distances`, `previous`, `visited`: O(V)
- Priority queue: O(V) in worst case
- Total: O(V)

Recurrence (intuitive per-node):
- Let T(k) be time after k settled nodes. Processing the (k+1)-th node does O(d) relaxations and up to O(d) heap operations:

  T(k+1) = T(k) + O(log V)

  with T(0) = O(1). Unrolling: T(V) = Σ_{i=1..V} O(log V) = O(V log V).

---

3) Bidirectional Dijkstra (`bidirectionalDijkstraGrid`) — used as the optimized baseline

Worst-case bounds are the same as standard Dijkstra, but the practical work is often much smaller.

Time (worst-case): O(V log V)
Space: O(V) (two distance arrays, two previous arrays, two visited arrays) → O(V)

Why it's faster in practice:
- Two searches run from start and goal. If they meet after exploring about V/2 nodes each, the total heap work ≈ 2 × (V/2 log (V/2)) ≈ V log V − lower-order constant. In many maps the explored region is roughly a sphere and bidirectional reduces explored area dramatically.

Meeting termination condition (informal): stop when min_forward + min_backward ≥ best_known_path_cost. This allows earlier stopping than single-sided where you must reach the goal.

Expected/typical time (heuristic): O((V/2) log V) per frontier → roughly half to an order-of-magnitude improvement in practice for many start/goal placements, but worst-case remains O(V log V).

Recurrence (informal): if F(k) is forward settled and B(k) backward settled, total steps ≈ F + B where F + B ≈ V_meet and each step costs O(log V). So

  T_bid(V) ≈ T_bid(V-1) + O(log V) with smaller constant factors compared to the one-sided run.

---

4) Time-expanded Dijkstra (dynamic wind) — `dijkstraGridTimeAware`

Model: each grid node is replicated for step = 0..H producing N = V × H states.

Time:
- Same Dijkstra bounds but on N states and edges between time-states: O((N + E_time) log N).
- E_time is O(d × N) ≈ O(N) (degree d constant). So time = O(N log N) = O(V H log(V H)).

Space:
- Distances array: O(N)
- `previous` map and `settledStates` set: O(N)
- Priority queue: O(N)
- Total: O(N) = O(V H)

Recurrence:
- Let N = V H and define T_timed(k) = T_timed(k-1) + O(log N) per processed state. Thus T_timed(N) = O(N log N).

Practical note: time-expanded search blows up quickly with horizon H; choose H conservatively.

---

5) Cost contributed by weight function
- `weightFn(from,to)` calls `geometricDistance` (O(1)) for baseline, or `computeEnergyEdgeCost`/`computeEnergyEdgeCostAtStep` (O(1)) for modified search. So edge weight evaluation is constant-time and doesn't change asymptotic complexity.
- `computeEnergyHeuristic` is O(1) and does not change worst-case time; in practice it reduces node expansions.

---

6) Combined end-to-end complexity for this project
- Baseline (bidirectional Dijkstra on grid):
  - Time: O(V log V) worst-case; typical much smaller due to bidirectional early stop
  - Space: O(V)
- Modified (energy-aware) — static wind (one-state per node):
  - Time: O(V log V)
  - Space: O(V)
- Modified (dynamic wind — time-expanded):
  - Time: O(V H log(V H))
  - Space: O(V H)
- Energy-Aware Theta* (any-angle):
  - Time: O((V + E) log V + L_total) where LOS checks are added to standard A*-style expansion.
  - Space: O(V)

  Notes:
  - Theta* behaves like A* with additional line-of-sight shortcutting and parent relaxation.
  - Heuristic guidance still reduces exploration, but each candidate parent connection requires a Bresenham LOS validation.
  - LOS traversal is O(k) per check, where k is the number of grid cells traversed.
  - Obstacle density influences runtime: dense blocked regions increase failed LOS attempts and local expansions.

Here L_total is the total number of Bresenham steps performed across all LOS checks.

Real-world recommendation: if dynamic wind is enabled, keep H = small constant × (r + c) or otherwise bounded to limit complexity.

---

7) Example numeric substitution
- For an 8×8 grid: V = 64, E ≈ 256
  - Single-state Dijkstra: O(64 log 64) ≈ 64 × 6 = ~384 (heap-op units)
  - Time-expanded with H = 20 → N = 1280; cost ≈ 1280 log 1280 ≈ 1280 × 11 = ~14k (heap-op units)

---

8) Final notes and trade-offs
- Asymptotic worst-case does not change by adding wind/altitude: the algorithm class remains Dijkstra on nonnegative weights.
- The dominant cost is heap operations (log factor). To accelerate further consider specialized integer-key radix heaps when weights are small integers, or A* with an admissible heuristic to reduce expansions when acceptable.

---

References and further reading
- Dijkstra's algorithm analysis (binary heap): O((V+E) log V)
- Bidirectional search: same worst-case but lower average expansions for many graphs
- Time-expanded graphs for time-dependent costs: multiply states by horizon H
