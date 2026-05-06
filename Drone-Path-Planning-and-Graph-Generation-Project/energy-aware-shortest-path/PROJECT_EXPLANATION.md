# Energy-Aware Shortest Path Project - Complete Explanation

## 🎯 What is This Project?

This is a **path-finding algorithm comparison** project. Imagine you're controlling a **drone** that needs to fly from one point to another on a grid-based map.

### The Problem:
- **Standard routing**: Just find the shortest distance (like Google Maps basic route)
- **Real drones**: Need to minimize **energy consumption**, not just distance

**Why?** A drone's battery matters! Flying into a headwind or climbing a mountain uses way more energy than flying with a tailwind or through valleys.

---

## 📊 The Grid Map

The map is an **8×8 grid** where each cell has:
- **Altitude** (height): 0 = sea level, 9 = mountain peak
- **Color intensity**: Light = low altitude, Dark = high altitude
- **Wind direction & strength**: Affects how hard it is to move
- **Obstacles**: Black cells the drone can't fly through

### Example Grid:
```
Start (S) ──────────────────── End (E)
     ↓                           ↓
[0.0][0.0][0.0][0.0][0.0][0.0][0.0][0.0]
[6.0][1.0][1.0][1.0][1.0][1.0][1.0][1.0]
[7.0][1.0][X  ][1.0][1.0][1.0][1.0][1.0]
[8.0][1.0][1.0][X  ][1.0][1.0][1.0][1.0]
...
```

---

## 🔧 How Movement Cost is Calculated

Every time the drone moves from one cell to an adjacent cell, it has a **cost** based on three factors:

### **Factor 1: Distance**
```
Distance = 1 (always, for 4-neighbor grid movement)
```
Each move from one cell to an adjacent cell = 1 unit of distance.

---

### **Factor 2: Wind Effect**
Wind tries to push the drone sideways. Flying **against** the wind costs more energy than flying **with** the wind.

**How it's calculated:**
```
Wind Effect = windStrength × (1 - dot_product)
```

**Breakdown:**
- `windStrength` = how strong the wind is (e.g., 1.5 = weak, 4.0 = strong)
- `dot_product` = how aligned your direction is with the wind direction
  - **dot = 1** (flying with wind): wind effect = windStrength × 0 = **0** (no cost!)
  - **dot = 0** (flying perpendicular): wind effect = windStrength × 1 = **windStrength** (medium cost)
  - **dot = -1** (flying against wind): wind effect = windStrength × 2 = **2 × windStrength** (maximum cost!)

**Example:**
```
Wind Direction: East (→)
Wind Strength: 1.5

Movement: North (↑)
dot_product = 0
Wind Effect = 1.5 × (1 - 0) = 1.5

Movement: East (→)  [WITH the wind]
dot_product = 1
Wind Effect = 1.5 × (1 - 1) = 0  [FREE!]

Movement: West (←)  [AGAINST the wind]
dot_product = -1
Wind Effect = 1.5 × (1 - (-1)) = 3.0  [EXPENSIVE!]
```

---

### **Factor 3: Altitude Cost**
Climbing mountains costs energy. Going downhill is cheaper.

**How it's calculated:**
```
climb = max(0, nextAltitude - currentAltitude)
descent = max(0, currentAltitude - nextAltitude)
altitude_cost = altitudeFactor × (climb + downhillFactor × descent)
```

**Breakdown:**
- `climb` = how much you're going UP (always ≥ 0)
- `descent` = how much you're going DOWN (always ≥ 0)
- `altitudeFactor` = how expensive climbing is (e.g., 1.0 = normal, 2.5 = very expensive)
- `downhillFactor` = discount for going downhill (default 0.35 = 35% cost of climbing)

**Example:**
```
Altitude Factor: 1.0
Downhill Factor: 0.35

Move UP from altitude 3 to 5:
climb = 5 - 3 = 2
descent = 0
cost = 1.0 × (2 + 0.35 × 0) = 2.0  [EXPENSIVE!]

Move DOWN from altitude 5 to 3:
climb = 0
descent = 5 - 3 = 2
cost = 1.0 × (0 + 0.35 × 2) = 0.7  [CHEAP!]
```

---

## 🚀 Total Edge Cost Formula

When the drone moves from cell A to cell B:

```
TOTAL COST = Distance + Wind Effect + Altitude Cost
           = 1.0 + windEffect + altitudeCost
```

This is the **weight** used in Dijkstra's algorithm for finding the shortest path.

---

## 🔄 Algorithm Comparison

### **Standard Dijkstra (Baseline - NOW BIDIRECTIONAL)**

**What it optimizes for:**
```
Total Cost = Distance ONLY
```

**How it works:**
1. Start from the start cell
2. Expand outward, visiting cells in order of shortest distance
3. BUT NOW (optimized): Also search backward from the goal
4. Two search fronts meet in the middle
5. Stop when they meet

**Why it's faster (bidirectional):**
- Instead of expanding across the entire grid from the start
- Two smaller expansions meet in the middle
- Fewer cells visited = faster = same shortest path result

---

### **Modified Dijkstra (Energy-Aware)**

**What it optimizes for:**
```
Total Cost = Distance + Wind Effect + Altitude Cost
```

**How it works:**
1. Same as Standard Dijkstra, BUT...
2. Edge weight = (distance + wind + altitude) instead of just distance
3. Picks routes that minimize ENERGY, not just distance
4. Might choose a LONGER path if it saves energy

**Example scenario:**
```
Shortest Distance Path:
- 7 cells
- All uphill against wind
- Total Energy: 63.0

Energy-Optimal Path:
- 10 cells (longer!)
- Through valleys with downhill
- Total Energy: 25.0  ✓ MUCH BETTER
```

---

## 📈 Side-by-Side Comparison

| Aspect | Standard Dijkstra | Modified Dijkstra |
|--------|-------------------|-------------------|
| **Optimizes For** | Distance only | Distance + Wind + Altitude |
| **Edge Weight** | 1.0 (always) | 1.0 + wind + altitude |
| **Path Selection** | Shortest route | Energy-cheapest route |
| **When Better** | Flat, no wind | Windy, mountainous terrain |
| **Search** | Bidirectional (optimized) | One-direction with time expansion |

---

## 🧮 Step-by-Step Example

**Grid:** 4×4
**Start:** (0,0)  **End:** (0,3)
**Wind:** East, strength 2.0
**Altitude Factor:** 1.0

```
Cell Layout:
(0,0)S → (0,1) → (0,2) → (0,3)E
  ↓        ↓       ↓        ↓
(1,0)    (1,1)   (1,2)   (1,3)
  ↓        ↓       ↓        ↓
(2,0)    (2,1)   (2,2)   (2,3)
  ↓        ↓       ↓        ↓
(3,0)    (3,1)   (3,2)   (3,3)

Altitudes:
[0][1][2][3]
[0][1][2][3]
[0][1][2][3]
[0][1][2][3]
```

### **Route 1: Direct Path (top row)**
```
(0,0) → (0,1) → (0,2) → (0,3)
Moving: East → East → East

Standard Dijkstra Cost:
Move 1: distance=1, wind=2×(1-1)=0, altitude=0 → Total=1
Move 2: distance=1, wind=0, altitude=1 → Total=2
Move 3: distance=1, wind=0, altitude=1 → Total=2
TOTAL PATH COST: 5.0
```

### **Route 2: Detour (bottom then right)**
```
(0,0) → (1,0) → (2,0) → (3,0) → (3,1) → (3,2) → (3,3)
Moving: South → South → South → East → East → East

Modified Dijkstra perspective:
- Going South: no wind resistance (perpendicular)
- Altitude stays at 0 (bottom row is flat)
- Final East movements with wind: neutral cost
- Total moves = 6, but energy-efficient!
```

---

## 🎯 Key Differences Explained Simply

### **Standard Dijkstra (Bidirectional)**
- "What's the shortest route?"
- Counts: Steps only
- Answer: "Go directly!" (7 steps)

### **Modified Dijkstra (Energy-Aware)**
- "What's the cheapest route?"
- Counts: Steps + Wind penalty + Climbing cost
- Answer: "Go around!" (Maybe 14 steps, but 60% less energy)

---

## 🔍 Real-World Example: Ridge Escape Scenario

**The Challenge:**
```
Start at (0,0) → Go to (7,7)
Left side is a mountain ridge (altitude 0→6→7→8→9)
Right side is flat (altitude all 1)
Wind blowing East
```

**Standard Dijkstra says:**
- "Just go left, climb the mountain, then cross right"
- Distance: 14 steps
- Energy: 37.74 (EXPENSIVE - lots of climbing + headwind)

**Modified Dijkstra says:**
- "Avoid the mountain entirely! Go flat on the right side"
- Distance: 14 steps (same!)
- Energy: 23.70 (SAVES 37% energy!)

**Why?**
- Flat terrain = no altitude cost
- Path avoids headwind zones when possible
- Total energy = much lower

---

## 📊 When Each Algorithm Wins

### **Standard Dijkstra wins when:**
- ✅ Flat terrain (no altitude)
- ✅ No wind
- ✅ Simple shortest-distance problem

### **Modified Dijkstra wins when:**
- ✅ Strong wind (3.0+)
- ✅ Steep terrain (altitude factor 2.0+)
- ✅ Mixed elevation landscape
- ✅ Real-world navigation (drones, ships, hikers)

### **Guarantee:**
Modified Dijkstra will **ALWAYS** find a path at least as good as Standard on energy cost, because it considers more factors.

---

## 🚁 The Optimization: Bidirectional Search

### **Old Way (One-sided Search):**
```
Start → ← ← ← ← ← ← ← ← ← End
[1][2][3][4][5][6][7][8][9] ... [thousands of cells]
Explores from start until reaches goal
```

### **New Way (Bidirectional Search):**
```
Start → ← ← ← ← ← ← ← ← ← ← End
[1][2][3][4][5] MEET [5][4][3][2][1]
Searches from BOTH directions
They meet in middle
Fewer cells visited = FASTER
Same path result = CORRECT
```

**Time saved: ~50-70% fewer cells explored**

---

## 📋 Project Files

| File | Purpose |
|------|---------|
| `js/algorithms.js` | Core algorithms (bidirectional Dijkstra + energy calculations) |
| `js/priorityQueue.js` | Min-heap priority queue for efficient node selection |
| `js/scenarios.js` | 4 test scenarios with different terrain/wind |
| `js/app.js` | Web UI controller |
| `index.html` | Interactive visualization interface |
| `scenario_runner.mjs` | Command-line test runner |

---

## 🎮 How to Use the Web Interface

1. **Load Scenario**: Pick from dropdown (Ridge Escape, Mountain Turn, Valley Detour, Headwind Valley)
2. **Adjust Parameters**: Change wind, altitude factor, enable dynamic wind
3. **Run All**: Click "Run All Algorithms"
4. **Compare Results**:
   - **Path Visualization**: Grid shows both paths in different colors
   - **Metrics Table**: See distance, energy, wind penalty, altitude penalty, execution time, nodes expanded
   - **Explanation**: Automatic text explanation of why one algorithm chose a different route

---

## 🏆 Summary

| Concept | Simple Explanation |
|---------|-------------------|
| **Standard Dijkstra** | Shortest distance, now searches from both ends |
| **Modified Dijkstra** | Cheapest energy considering wind and terrain |
| **Distance Cost** | Always 1 per move |
| **Wind Cost** | How much you fight the wind (high if against, zero if tailwind) |
| **Altitude Cost** | Climbing is expensive, downhill is cheap |
| **When to use each** | Standard for simple cases, Modified for real-world navigation |

---

## 💡 Key Insight

**This project proves:** In the real world, shortest ≠ cheapest.

A drone pilot would choose the energy-efficient path (Modified Dijkstra) over the shortest path (Standard Dijkstra) because it means:
- ✅ Longer flight range
- ✅ Lower battery drain
- ✅ Can complete more missions per charge
- ✅ More practical navigation

That's why the **Modified Dijkstra is the better algorithm** for real-world drone path planning!

