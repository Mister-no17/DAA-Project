# Time Complexity Derivation and Recurrence Relations

This document derives the running times for the algorithms used in this project and clarifies where a recurrence relation is mathematically appropriate.

## 1) Problem Model and Symbols

Let:
- R = number of grid rows
- C = number of grid columns
- V = R * C = number of grid cells (graph vertices)
- E = number of directed adjacency edges

In this implementation, movement is 4-directional (up, down, left, right).
- Each cell has at most 4 outgoing neighbors.
- So E <= 4V (directed counting used by the loops in code).
- Therefore E = O(V) for this grid graph.

Useful substitution for this project:
- V = R * C
- E = O(R * C)

## 2) Priority Queue Cost Used by Dijkstra Variants

The project uses a binary min-heap priority queue.
- push: O(log M)
- pop: O(log M)
where M is current heap size (at most proportional to V or state count).

This is why Dijkstra terms include a log factor.

## 3) Standard Dijkstra on the Grid

Implementation path: runStandardDijkstra -> dijkstraGrid.

### 3.1 Work decomposition

1. Initialization over arrays of size V:
- distances, previous, visited
- Cost: O(V)

2. Main loop:
- Each settled vertex is popped once (ignoring stale entries, total still bounded by edge relax process).
- For each settled vertex, its outgoing edges are scanned.
- Total edge scans across entire run: O(E).

3. Relaxation events:
- A successful relaxation pushes a candidate in heap.
- Number of successful relaxations is O(E) upper bound.
- Each push/pop is O(log V) in the usual analysis.

### 3.2 Total

Combining:
- O(V) initialization
- O(E log V) from edge-driven pushes
- O(V log V) from vertex extractions

Total:
- O((V + E) log V)

Because E = O(V) on this 4-neighbor grid:
- O((V + E) log V) = O(V log V)
- In R, C form: O(R * C * log(R * C))

### 3.3 Recurrence relation status

There is no standard divide-and-conquer recurrence used to analyze Dijkstra.
Dijkstra is typically analyzed by counting graph operations plus priority queue costs.

So for presentation:
- Recurrence relation: Not standard / not the primary analysis tool.
- Correct analysis: operation-count asymptotic bound above.

## 4) Modified Dijkstra (Static Wind)

Implementation path: runModifiedDijkstra (static mode) -> dijkstraGrid with different weightFn.

Only edge weights change (distance + wind + altitude), but:
- Neighbor structure is unchanged.
- Number of relax checks is unchanged asymptotically.
- Heap operation pattern is unchanged asymptotically.

Therefore the same derivation applies:
- O((V + E) log V)
- With grid substitution: O(V log V) = O(R * C * log(R * C))

Recurrence status is same as standard Dijkstra:
- No standard recurrence-based complexity proof.

## 5) Bellman-Ford DP (Static Wind)

Implementation path: runBellmanFord (static mode) -> bellmanFordGrid.

This is the case where a true DP recurrence is standard.

### 5.1 DP recurrence (valid and standard)

Define dp[k][v] = minimum cost to reach vertex v using at most k edges.

Recurrence:
- dp[0][s] = 0, dp[0][v != s] = +infinity
- dp[k][v] = min( dp[k-1][v], min over (u,v) in E of (dp[k-1][u] + w(u,v)) )

Bellman-Ford performs these relaxations iteratively up to k = V - 1.

### 5.2 Time derivation

Outer iterations:
- At most V - 1 rounds.

Per round work:
- Scan all vertices and their outgoing edges.
- Equivalent to O(E) relax attempts per round.

Total:
- O((V - 1) * E) = O(VE)

In this code, there is early stopping when no update occurs in a round.
- Best-case with early stop after first pass: O(E)
- General/worst-case: O(VE)

Grid substitution with E = O(V):
- O(VE) becomes O(V^2)
- In R, C: O((R * C)^2)

## 6) Dynamic Wind Mode (Time-Expanded State Space)

Implementation paths:
- runModifiedDijkstra (dynamic mode) -> dijkstraGridTimeAware
- runBellmanFord (dynamic mode) currently also falls back to dijkstraGridTimeAware

So in dynamic mode, both displayed algorithms are solved by shortest path on a time-expanded graph.

### 6.1 Time-expanded graph size

Let H = horizon (maxSteps).

State is (vertex, step), so:
- V_t = (H + 1) * V

Each state has up to 4 outgoing transitions to next step, so:
- E_t = O(H * E)

### 6.2 Dijkstra on expanded graph

Cost:
- O((V_t + E_t) log V_t)

Substitute:
- O(((H + 1)V + H E) * log((H + 1)V))

For grid where E = O(V):
- O(H * V * log(HV))

If H itself is proportional to V (as in code horizon rule using a multiple of rows*cols):
- O(V^2 log V) order (up to constants and log(HV) vs log V).

### 6.3 Recurrence status in dynamic mode

Important for viva/teacher discussion:
- A DP transition equation can be written for layered states.
- But this implementation solves the layered graph with Dijkstra, not with Bellman-Ford recurrence iterations.
- So the recurrence is not the primary executed method in dynamic mode.

A valid transition relation for the state graph is:
- dist[t+1][v] = min over u in Pred(v) of (dist[t][u] + w_t(u,v))

However, complexity reported for this code path should follow the chosen solver:
- O((V_t + E_t) log V_t)

## 7) Best / Average / Worst Case Summary (As Presented in UI)

### Standard Dijkstra (binary heap)
- Best: O((V + E) log V)
- Average: O((V + E) log V)
- Worst: O((V + E) log V)

### Modified Dijkstra (static wind)
- Best: O((V + E) log V)
- Average: O((V + E) log V)
- Worst: O((V + E) log V)

### Bellman-Ford DP (static wind)
- Best (early stopping): O(E)
- Average: O(VE)
- Worst: O(VE)

### Time-expanded Dijkstra fallback (dynamic wind mode)
- Best: O((V_t + E_t) log V_t)
- Average: O((V_t + E_t) log V_t)
- Worst: O((V_t + E_t) log V_t)
where V_t = (H + 1)V and E_t = O(H E).

## 8) "When can we calculate recurrence relation?"

You can state this rule clearly:

1. Recurrence is natural when the algorithm is defined by smaller subproblems (DP, divide-and-conquer).
2. For Bellman-Ford DP, recurrence is standard and correct.
3. For Dijkstra, recurrence is generally not the standard tool; operation counting with heap costs is standard.
4. For dynamic-wind layered states, a transition relation exists, but if solved by Dijkstra, complexity should be derived from graph + heap operations on expanded states.

So your presentation is academically safe if you say:
- "Recurrence shown where algorithm is DP by definition."
- "For greedy shortest-path (Dijkstra variants), we use operation-count complexity instead of recurrence."

## 9) Direct Mapping to This Codebase

- Standard Dijkstra complexity source: dijkstraGrid
- Modified static complexity source: dijkstraGrid with energy weight
- Bellman-Ford recurrence and complexity source: bellmanFordGrid
- Dynamic mode complexity source for both modified and DP-labeled run: dijkstraGridTimeAware

This mapping is exactly why recurrence applicability is different per mode.
