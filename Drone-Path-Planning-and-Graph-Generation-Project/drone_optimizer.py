"""3D drone delivery route optimization with physically accurate cost modelling.

This module implements Dijkstra, A*, and Greedy Best-First Search on a
deterministic 3D grid environment using a composite physical cost function that
combines distance, gravitational climb/descent energy, wind alignment, and
crosswind correction effort.
"""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass
from heapq import heappop, heappush
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Set, Tuple

import matplotlib
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================

# Grid dimensions in nodes.
X_MAX = 20
Y_MAX = 20
Z_MAX = 8

# Real-world scale of one grid step.
X_SCALE_METRES = 10.0  # metres per x-step
Y_SCALE_METRES = 10.0  # metres per y-step
Z_SCALE_METRES = 15.0  # metres per altitude layer

# Flight envelope.
GROUND_LEVEL_Z = 0
MIN_FLIGHT_Z = 1
MAX_FLIGHT_Z = Z_MAX - 1

# Drone physics constants.
DRONE_MASS_KG = 2.5  # kilograms
GRAVITY_MPS2 = 9.81  # metres per second squared
REGEN_FACTOR = -0.3  # descending recovers 30% of gravitational energy
DRONE_SPEED_MPS = 15.0  # cruise speed in metres per second
WIND_COEFF = 0.5  # aerodynamic alignment tuning constant
CROSS_COEFF = 0.1  # quadratic crosswind correction tuning constant
MIN_EDGE_WEIGHT = 0.01  # prevents non-positive edges

# Plotting configuration.
PLOT_DPI = 150
WIND_ARROW_SCALE = 8.0
SHOW_PAUSE_SECONDS = 0.1

# Paths to generated artefacts.
ROUTE_PLOT_PATH = Path("route_3d.png")
COST_BREAKDOWN_PATH = Path("cost_breakdown.png")
ALGO_COMPARISON_PATH = Path("algo_comparison.png")
ALTITUDE_PROFILE_PATH = Path("altitude_profile.png")
ANALYSIS_PATH = Path("analysis.txt")

# Core domain data.
Node = Tuple[int, int, int]
WindVector = Tuple[float, float, float]
ObstacleSet = Set[Node]

WAYPOINTS: Dict[str, Node] = {
    "Depot": (1, 1, 2),
    "D1": (18, 3, 3),
    "D2": (10, 15, 4),
    "D3": (16, 18, 2),
}

DELIVERY_NAMES = ("D1", "D2", "D3")
OPTIMALITY_LABELS = {
    "Dijkstra": "Guaranteed",
    "A*": "Guaranteed",
    "Greedy BFS": "NOT guaranteed",
}

WIND_SCENARIOS: List[Tuple[str, WindVector, str]] = [
    ("Calm", (0.0, 0.0, 0.0), "No wind - baseline comparison"),
    ("Headwind NE", (3.0, 3.0, 0.5), "Moderate NE wind with updraft"),
    ("Crosswind+Down", (-5.0, 2.0, -1.0), "Strong crosswind with downdraft"),
]

NEIGHBOUR_DELTAS: Tuple[Node, ...] = tuple(
    (dx, dy, dz)
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dz in (-1, 0, 1)
    if (dx, dy, dz) != (0, 0, 0)
)


@dataclass
class MissionResult:
    """Aggregated mission output for one algorithm and one wind scenario."""

    algorithm_name: str
    mission_path: List[Node]
    total_cost: float
    nodes_explored: int
    time_ms: float
    leg_paths: List[List[Node]]
    leg_costs: List[float]
    edge_components: List[Dict[str, float]]

    @property
    def path_length(self) -> int:
        """Return the number of nodes in the concatenated mission path."""

        return len(self.mission_path)


@dataclass
class ScenarioResult:
    """All algorithm outputs for a single wind scenario."""

    name: str
    wind: WindVector
    description: str
    results: Dict[str, MissionResult]


AlgorithmFunction = Callable[
    [Node, Node, ObstacleSet, WindVector], Tuple[List[Node], float, int, float]
]


# ============================================================================
# OBSTACLE GENERATION
# ============================================================================

def is_within_bounds(node: Node) -> bool:
    """Return True if a node lies within grid limits and the flight envelope."""

    x, y, z = node
    return 0 <= x < X_MAX and 0 <= y < Y_MAX and MIN_FLIGHT_Z <= z <= MAX_FLIGHT_Z


def generate_obstacles() -> ObstacleSet:
    """Generate a deterministic obstacle set for the 3D grid.

    Returns:
        Set of blocked nodes expressed as integer grid coordinates.
    """

    random.seed(42)
    obstacles: ObstacleSet = set()
    protected_nodes = set(WAYPOINTS.values())

    footprints = [(x, y) for x in (2, 6, 10, 14, 18) for y in (2, 6, 10, 14, 18)]
    random.shuffle(footprints)

    selected_footprints: List[Tuple[int, int]] = []
    for footprint in footprints:
        if footprint in {(node[0], node[1]) for node in protected_nodes}:
            continue
        if all(math.dist(footprint, other) >= 3.0 for other in selected_footprints):
            selected_footprints.append(footprint)
        if len(selected_footprints) == 12:
            break

    for x, y in selected_footprints:
        height = random.randint(2, 6)
        for z in range(MIN_FLIGHT_Z, height + 1):
            obstacles.add((x, y, z))

    for x in range(8, 15):
        for y in range(8, 15):
            obstacles.add((x, y, 5))

    for x in range(12, 18):
        for y in range(4, 11):
            obstacles.add((x, y, 3))

    for x in range(3, 9):
        base_z = math.floor(x / 2)
        for z in (base_z, base_z + 1):
            if MIN_FLIGHT_Z <= z <= MAX_FLIGHT_Z:
                obstacles.add((x, x, z))

    for node in protected_nodes:
        assert node not in obstacles, f"Protected node blocked by obstacle: {node}"
        assert is_within_bounds(node), f"Protected node out of bounds: {node}"

    return obstacles


# ============================================================================
# COST FUNCTION
# ============================================================================

def node_to_metres(node: Node) -> Tuple[float, float, float]:
    """Convert a grid node into physical coordinates in metres."""

    x, y, z = node
    return (x * X_SCALE_METRES, y * Y_SCALE_METRES, z * Z_SCALE_METRES)


def flight_vector_metres(u: Node, v: Node) -> np.ndarray:
    """Return the movement vector from u to v in metres."""

    dx = (v[0] - u[0]) * X_SCALE_METRES
    dy = (v[1] - u[1]) * Y_SCALE_METRES
    dz = (v[2] - u[2]) * Z_SCALE_METRES
    return np.array((dx, dy, dz), dtype=float)


def straight_line_distance_metres(a: Node, b: Node) -> float:
    """Return the 3D Euclidean distance between two nodes in metres."""

    return float(np.linalg.norm(flight_vector_metres(a, b)))


def edge_cost_components(u: Node, v: Node, wind: WindVector) -> Dict[str, float]:
    """Compute all physical cost components for one directed edge.

    Args:
        u: Start node as integer grid coordinates.
        v: End node as integer grid coordinates.
        wind: Wind vector (wx, wy, wz) in metres per second.

    Returns:
        Dictionary containing all edge cost components in project cost units.
    """

    flight_vec = flight_vector_metres(u, v)
    euclidean_distance = float(np.linalg.norm(flight_vec))
    if euclidean_distance <= 0.0:
        raise ValueError("Edge cost requested for zero-length move.")

    delta_h = (v[2] - u[2]) * Z_SCALE_METRES
    if delta_h > 0.0:
        altitude_cost = DRONE_MASS_KG * GRAVITY_MPS2 * delta_h
    elif delta_h < 0.0:
        altitude_cost = REGEN_FACTOR * DRONE_MASS_KG * GRAVITY_MPS2 * abs(delta_h)
    else:
        altitude_cost = 0.0

    flight_unit = flight_vec / euclidean_distance
    wind_vec = np.array(wind, dtype=float)
    relative_wind = wind_vec - (flight_unit * DRONE_SPEED_MPS)

    wind_dot = float(np.dot(wind_vec, flight_unit))
    wind_cost = -WIND_COEFF * wind_dot * euclidean_distance

    crosswind_vec = wind_vec - (wind_dot * flight_unit)
    crosswind_mag = float(np.linalg.norm(crosswind_vec))
    crosswind_cost = CROSS_COEFF * (crosswind_mag ** 2) * euclidean_distance

    total_cost = euclidean_distance + altitude_cost + wind_cost + crosswind_cost

    return {
        "base_distance_cost": euclidean_distance,
        "altitude_cost": altitude_cost,
        "wind_cost": wind_cost,
        "crosswind_cost": crosswind_cost,
        "raw_total_cost": total_cost,
        "clamped_total_cost": max(total_cost, MIN_EDGE_WEIGHT),
        "delta_h_metres": delta_h,
        "crosswind_mag_mps": crosswind_mag,
        "wind_dot_mps": wind_dot,
        "relative_wind_mag_mps": float(np.linalg.norm(relative_wind)),
    }


def edge_weight(u: Node, v: Node, wind: WindVector) -> float:
    """Return the clamped composite edge cost from u to v."""

    return edge_cost_components(u, v, wind)["clamped_total_cost"]


# ============================================================================
# GRAPH UTILITIES
# ============================================================================

def is_valid(node: Node, obstacles: ObstacleSet) -> bool:
    """Return True if the node is flyable and not blocked."""

    return is_within_bounds(node) and node not in obstacles


def get_neighbours(node: Node, obstacles: ObstacleSet) -> List[Node]:
    """Return all valid 3D Moore-neighbour nodes for the given node."""

    neighbours: List[Node] = []
    x, y, z = node
    for dx, dy, dz in NEIGHBOUR_DELTAS:
        neighbour = (x + dx, y + dy, z + dz)
        if is_valid(neighbour, obstacles):
            neighbours.append(neighbour)
    return neighbours


def heuristic(node: Node, goal: Node) -> float:
    """Return the straight-line 3D Euclidean distance in metres."""

    return straight_line_distance_metres(node, goal)


def a_star_lower_bound(node: Node, goal: Node, wind: WindVector) -> float:
    """Return an admissible lower bound for the remaining physical path cost.

    The lower bound combines:
    - straight-line distance, which lower-bounds total travelled distance
    - the cheapest possible net altitude contribution for the required altitude change
    - the exact path-independent wind alignment term for a constant wind field
    - a minimum edge clamp bound based on the least number of required steps
    """

    straight_line = straight_line_distance_metres(node, goal)
    delta_h = (goal[2] - node[2]) * Z_SCALE_METRES
    if delta_h > 0.0:
        altitude_lower_bound = DRONE_MASS_KG * GRAVITY_MPS2 * delta_h
    elif delta_h < 0.0:
        altitude_lower_bound = REGEN_FACTOR * DRONE_MASS_KG * GRAVITY_MPS2 * abs(delta_h)
    else:
        altitude_lower_bound = 0.0

    wind_vec = np.array(wind, dtype=float)
    net_displacement = flight_vector_metres(node, goal)
    wind_term = -WIND_COEFF * float(np.dot(wind_vec, net_displacement))

    dx = abs(goal[0] - node[0])
    dy = abs(goal[1] - node[1])
    dz = abs(goal[2] - node[2])
    minimum_required_steps = max(dx, dy, dz)
    clamp_lower_bound = minimum_required_steps * MIN_EDGE_WEIGHT

    return max(straight_line + altitude_lower_bound + wind_term, clamp_lower_bound, 0.0)


def reconstruct_path(
    predecessors: Dict[Node, Optional[Node]], start: Node, goal: Node
) -> List[Node]:
    """Reconstruct a path from predecessor links."""

    if goal not in predecessors:
        return []

    current: Optional[Node] = goal
    path: List[Node] = []
    while current is not None:
        path.append(current)
        current = predecessors[current]
    path.reverse()

    if not path or path[0] != start:
        return []
    return path


def compute_path_cost(path: Sequence[Node], wind: WindVector) -> float:
    """Return the total composite cost of a path under a wind scenario."""

    if len(path) < 2:
        return 0.0
    return sum(edge_weight(path[i], path[i + 1], wind) for i in range(len(path) - 1))


def collect_path_edge_components(
    path: Sequence[Node], wind: WindVector
) -> List[Dict[str, float]]:
    """Return physical cost components for every edge in a path."""

    components: List[Dict[str, float]] = []
    for index in range(len(path) - 1):
        edge_data = edge_cost_components(path[index], path[index + 1], wind)
        edge_data["edge_index"] = float(index)
        components.append(edge_data)
    return components


# ============================================================================
# DIJKSTRA
# ============================================================================

def dijkstra(
    start: Node, goal: Node, obstacles: ObstacleSet, wind: WindVector
) -> Tuple[List[Node], float, int, float]:
    """Run Dijkstra's algorithm on the 3D grid.

    Returns:
        Tuple of (path, path_cost, nodes_explored, execution_time_ms).
    """

    start_time = time.perf_counter()
    queue: List[Tuple[float, Node]] = [(0.0, start)]
    distances: Dict[Node, float] = {start: 0.0}
    predecessors: Dict[Node, Optional[Node]] = {start: None}
    expanded: Set[Node] = set()

    while queue:
        current_cost, current = heappop(queue)
        if current in expanded:
            continue
        expanded.add(current)

        if current == goal:
            break

        for neighbour in get_neighbours(current, obstacles):
            new_cost = current_cost + edge_weight(current, neighbour, wind)
            if new_cost < distances.get(neighbour, math.inf):
                distances[neighbour] = new_cost
                predecessors[neighbour] = current
                heappush(queue, (new_cost, neighbour))

    time_ms = (time.perf_counter() - start_time) * 1000.0
    path = reconstruct_path(predecessors, start, goal)
    if not path:
        return [], math.inf, len(expanded), time_ms
    return path, distances[goal], len(expanded), time_ms


# ============================================================================
# A_STAR
# ============================================================================

def a_star(
    start: Node, goal: Node, obstacles: ObstacleSet, wind: WindVector
) -> Tuple[List[Node], float, int, float]:
    """Run A* search on the 3D grid with an admissible heuristic.

    Returns:
        Tuple of (path, path_cost, nodes_explored, execution_time_ms).
    """

    start_time = time.perf_counter()
    queue: List[Tuple[float, float, float, Node]] = [
        (a_star_lower_bound(start, goal, wind), heuristic(start, goal), 0.0, start)
    ]
    g_scores: Dict[Node, float] = {start: 0.0}
    predecessors: Dict[Node, Optional[Node]] = {start: None}
    expanded_unique: Set[Node] = set()

    while queue:
        _, _, current_g, current = heappop(queue)
        if current_g > g_scores.get(current, math.inf):
            continue
        expanded_unique.add(current)

        if current == goal:
            break

        for neighbour in get_neighbours(current, obstacles):
            tentative_g = current_g + edge_weight(current, neighbour, wind)
            if tentative_g < g_scores.get(neighbour, math.inf):
                g_scores[neighbour] = tentative_g
                predecessors[neighbour] = current
                euclidean_tiebreak = heuristic(neighbour, goal)
                f_score = tentative_g + a_star_lower_bound(neighbour, goal, wind)
                heappush(queue, (f_score, euclidean_tiebreak, tentative_g, neighbour))

    time_ms = (time.perf_counter() - start_time) * 1000.0
    path = reconstruct_path(predecessors, start, goal)
    if not path:
        return [], math.inf, len(expanded_unique), time_ms
    return path, g_scores[goal], len(expanded_unique), time_ms


# ============================================================================
# GREEDY_BFS
# ============================================================================

def greedy_bfs(
    start: Node, goal: Node, obstacles: ObstacleSet, wind: WindVector
) -> Tuple[List[Node], float, int, float]:
    """Run Greedy Best-First Search on the 3D grid.

    Returns:
        Tuple of (path, actual_path_cost, nodes_explored, execution_time_ms).
    """

    start_time = time.perf_counter()
    queue: List[Tuple[float, Node]] = [(heuristic(start, goal), start)]
    predecessors: Dict[Node, Optional[Node]] = {start: None}
    discovered: Set[Node] = {start}
    expanded: Set[Node] = set()

    while queue:
        _, current = heappop(queue)
        if current in expanded:
            continue
        expanded.add(current)

        if current == goal:
            break

        for neighbour in get_neighbours(current, obstacles):
            if neighbour in discovered:
                continue
            discovered.add(neighbour)
            predecessors[neighbour] = current
            heappush(queue, (heuristic(neighbour, goal), neighbour))

    time_ms = (time.perf_counter() - start_time) * 1000.0
    path = reconstruct_path(predecessors, start, goal)
    if not path:
        return [], math.inf, len(expanded), time_ms
    return path, compute_path_cost(path, wind), len(expanded), time_ms


# ============================================================================
# TSP_GREEDY
# ============================================================================

def tsp_greedy(waypoints: Optional[Dict[str, Node]] = None) -> List[str]:
    """Return the greedy nearest-neighbour multi-stop visit order."""

    points = waypoints or WAYPOINTS
    current = "Depot"
    unvisited = list(DELIVERY_NAMES)
    order = [current]

    while unvisited:
        next_stop = min(
            unvisited,
            key=lambda name: straight_line_distance_metres(points[current], points[name]),
        )
        order.append(next_stop)
        unvisited.remove(next_stop)
        current = next_stop

    order.append("Depot")
    return order


def run_mission(
    algorithm_name: str,
    algorithm: AlgorithmFunction,
    visit_order: Sequence[str],
    obstacles: ObstacleSet,
    wind: WindVector,
) -> MissionResult:
    """Run one search algorithm across all mission legs and aggregate metrics."""

    mission_path: List[Node] = []
    leg_paths: List[List[Node]] = []
    leg_costs: List[float] = []
    total_cost = 0.0
    total_nodes_explored = 0
    total_time_ms = 0.0

    for start_name, goal_name in zip(visit_order[:-1], visit_order[1:]):
        start_node = WAYPOINTS[start_name]
        goal_node = WAYPOINTS[goal_name]
        path, cost, nodes_explored, time_ms = algorithm(start_node, goal_node, obstacles, wind)
        if not path:
            raise RuntimeError(
                f"{algorithm_name} could not find a path from {start_name} to {goal_name}."
            )

        if mission_path:
            mission_path.extend(path[1:])
        else:
            mission_path.extend(path)

        leg_paths.append(path)
        leg_costs.append(cost)
        total_cost += cost
        total_nodes_explored += nodes_explored
        total_time_ms += time_ms

    edge_components = collect_path_edge_components(mission_path, wind)
    return MissionResult(
        algorithm_name=algorithm_name,
        mission_path=mission_path,
        total_cost=total_cost,
        nodes_explored=total_nodes_explored,
        time_ms=total_time_ms,
        leg_paths=leg_paths,
        leg_costs=leg_costs,
        edge_components=edge_components,
    )


# ============================================================================
# VISUALIZATION
# ============================================================================

def prepare_figure_for_output(fig: plt.Figure, output_path: Path) -> None:
    """Save a figure, show it non-blocking, then close it."""

    fig.tight_layout()
    fig.savefig(output_path, dpi=PLOT_DPI, bbox_inches="tight")
    backend_name = matplotlib.get_backend().lower()
    if "agg" not in backend_name:
        plt.show(block=False)
        plt.pause(SHOW_PAUSE_SECONDS)
    plt.close(fig)


def plot_mission_path_3d(ax: plt.Axes, path: Sequence[Node], **plot_kwargs: object) -> None:
    """Plot a path on a 3D axis using metre-scaled coordinates."""

    if not path:
        return
    coordinates = np.array([node_to_metres(node) for node in path], dtype=float)
    ax.plot(coordinates[:, 0], coordinates[:, 1], coordinates[:, 2], **plot_kwargs)


def create_route_plot(scenario_results: Sequence[ScenarioResult], obstacles: ObstacleSet) -> None:
    """Create the 3D multi-scenario route visualization."""

    obstacle_coords = np.array([node_to_metres(node) for node in sorted(obstacles)], dtype=float)
    fig = plt.figure(figsize=(20, 6))

    for index, scenario in enumerate(scenario_results, start=1):
        ax = fig.add_subplot(1, 3, index, projection="3d")

        if len(obstacle_coords) > 0:
            ax.scatter(
                obstacle_coords[:, 0],
                obstacle_coords[:, 1],
                obstacle_coords[:, 2],
                c="grey",
                marker="s",
                alpha=0.15,
                s=35,
                label="Obstacles",
            )

        plot_mission_path_3d(
            ax,
            scenario.results["A*"].mission_path,
            color="blue",
            linestyle="-",
            linewidth=2.0,
            marker="o",
            markersize=2.5,
            label="A*",
        )
        plot_mission_path_3d(
            ax,
            scenario.results["Dijkstra"].mission_path,
            color="red",
            linestyle="--",
            linewidth=1.8,
            label="Dijkstra",
        )
        plot_mission_path_3d(
            ax,
            scenario.results["Greedy BFS"].mission_path,
            color="green",
            linestyle=":",
            linewidth=2.0,
            label="Greedy BFS",
        )

        depot_x, depot_y, depot_z = node_to_metres(WAYPOINTS["Depot"])
        ax.scatter(
            [depot_x],
            [depot_y],
            [depot_z],
            c="green",
            marker="*",
            s=150,
            label="Depot",
        )

        delivery_coords = np.array(
            [node_to_metres(WAYPOINTS[name]) for name in DELIVERY_NAMES], dtype=float
        )
        ax.scatter(
            delivery_coords[:, 0],
            delivery_coords[:, 1],
            delivery_coords[:, 2],
            c="orange",
            marker="D",
            s=60,
            label="Deliveries",
        )

        center_x = ((X_MAX - 1) * X_SCALE_METRES) / 2.0
        center_y = ((Y_MAX - 1) * Y_SCALE_METRES) / 2.0
        center_z = ((MIN_FLIGHT_Z + MAX_FLIGHT_Z) * Z_SCALE_METRES) / 2.0
        wind = scenario.wind
        ax.quiver(
            center_x,
            center_y,
            center_z,
            wind[0] * WIND_ARROW_SCALE,
            wind[1] * WIND_ARROW_SCALE,
            wind[2] * WIND_ARROW_SCALE,
            color="purple",
            linewidth=2.0,
            arrow_length_ratio=0.15,
        )

        ax.set_xlim(0.0, (X_MAX - 1) * X_SCALE_METRES)
        ax.set_ylim(0.0, (Y_MAX - 1) * Y_SCALE_METRES)
        ax.set_zlim(MIN_FLIGHT_Z * Z_SCALE_METRES, MAX_FLIGHT_Z * Z_SCALE_METRES)
        ax.set_xlabel("X (metres)")
        ax.set_ylabel("Y (metres)")
        ax.set_zlabel("Z - Altitude (metres)")
        ax.set_title(f"3D Drone Route Optimization - {scenario.name}")
        ax.legend(loc="upper left", fontsize=8)

    prepare_figure_for_output(fig, ROUTE_PLOT_PATH)


def create_cost_breakdown_plot(scenario_results: Sequence[ScenarioResult]) -> None:
    """Create stacked per-edge cost breakdown charts for the A* mission path."""

    fig, axes = plt.subplots(3, 1, figsize=(16, 12), sharex=False)
    component_labels = (
        ("Base Distance", "base_distance_cost", "#4C78A8"),
        ("Altitude", "altitude_cost", "#F58518"),
        ("Wind", "wind_cost", "#54A24B"),
        ("Crosswind", "crosswind_cost", "#E45756"),
    )

    for ax, scenario in zip(axes, scenario_results):
        components = scenario.results["A*"].edge_components
        edge_indices = np.arange(len(components))
        positive_bottom = np.zeros(len(components))
        negative_bottom = np.zeros(len(components))

        for label, key, color in component_labels:
            values = np.array([component[key] for component in components], dtype=float)
            bottoms = np.where(values >= 0.0, positive_bottom, negative_bottom)
            ax.bar(edge_indices, values, bottom=bottoms, color=color, label=label)
            positive_bottom = np.where(values >= 0.0, positive_bottom + values, positive_bottom)
            negative_bottom = np.where(values < 0.0, negative_bottom + values, negative_bottom)

        ax.axhline(0.0, color="black", linewidth=0.8)
        ax.set_title(f"A* Edge Cost Breakdown - {scenario.name}")
        ax.set_xlabel("Edge Index")
        ax.set_ylabel("Cost Units")
        ax.legend(loc="upper right")

    prepare_figure_for_output(fig, COST_BREAKDOWN_PATH)


def create_algorithm_comparison_plot(scenario_results: Sequence[ScenarioResult]) -> None:
    """Create grouped bar charts for cost and nodes explored."""

    algorithms = ["Dijkstra", "A*", "Greedy BFS"]
    colors = {"Dijkstra": "red", "A*": "blue", "Greedy BFS": "green"}
    x_positions = np.arange(len(scenario_results))
    width = 0.25

    fig, axes = plt.subplots(1, 2, figsize=(16, 6))

    for offset, algorithm in enumerate(algorithms):
        cost_values = [scenario.results[algorithm].total_cost for scenario in scenario_results]
        node_values = [
            scenario.results[algorithm].nodes_explored for scenario in scenario_results
        ]
        shifted_positions = x_positions + (offset - 1) * width

        axes[0].bar(
            shifted_positions,
            cost_values,
            width=width,
            color=colors[algorithm],
            label=algorithm,
        )
        axes[1].bar(
            shifted_positions,
            node_values,
            width=width,
            color=colors[algorithm],
            label=algorithm,
        )

    scenario_names = [scenario.name for scenario in scenario_results]
    for ax, title, ylabel in (
        (axes[0], "Total Mission Cost by Algorithm", "Cost Units"),
        (axes[1], "Nodes Explored by Algorithm", "Nodes Explored"),
    ):
        ax.set_xticks(x_positions)
        ax.set_xticklabels(scenario_names)
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.legend(loc="upper right")

    prepare_figure_for_output(fig, ALGO_COMPARISON_PATH)


def create_altitude_profile_plot(scenario_results: Sequence[ScenarioResult]) -> None:
    """Create altitude profile charts for the A* mission path."""

    fig, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=False)
    no_fly_altitude = 5 * Z_SCALE_METRES

    for ax, scenario in zip(axes, scenario_results):
        path = scenario.results["A*"].mission_path
        altitudes = [node[2] * Z_SCALE_METRES for node in path]
        steps = np.arange(len(path))

        ax.plot(steps, altitudes, color="blue", linewidth=2.0, label="A* Altitude")
        ax.axhline(
            no_fly_altitude,
            color="red",
            linestyle="--",
            linewidth=1.5,
            label="No-fly slab",
        )
        ax.set_title(f"A* Altitude Profile - {scenario.name}")
        ax.set_xlabel("Path Step Index")
        ax.set_ylabel("Altitude (metres)")
        ax.legend(loc="upper right")

    prepare_figure_for_output(fig, ALTITUDE_PROFILE_PATH)


# ============================================================================
# REPORTING
# ============================================================================

def format_comparison_table(scenario: ScenarioResult) -> str:
    """Create an ASCII comparison table for one wind scenario."""

    metric_width = 24
    col_width = 18
    header = (
        f"{'Metric':<{metric_width}}"
        f"{'Dijkstra':>{col_width}}"
        f"{'A*':>{col_width}}"
        f"{'Greedy BFS':>{col_width}}"
    )
    separator = "-" * len(header)

    rows = [
        (
            "Total Path Cost (units)",
            f"{scenario.results['Dijkstra'].total_cost:.2f}",
            f"{scenario.results['A*'].total_cost:.2f}",
            f"{scenario.results['Greedy BFS'].total_cost:.2f}",
        ),
        (
            "Nodes Explored",
            str(scenario.results["Dijkstra"].nodes_explored),
            str(scenario.results["A*"].nodes_explored),
            str(scenario.results["Greedy BFS"].nodes_explored),
        ),
        (
            "Path Length (nodes)",
            str(scenario.results["Dijkstra"].path_length),
            str(scenario.results["A*"].path_length),
            str(scenario.results["Greedy BFS"].path_length),
        ),
        (
            "Execution Time (ms)",
            f"{scenario.results['Dijkstra'].time_ms:.2f}",
            f"{scenario.results['A*'].time_ms:.2f}",
            f"{scenario.results['Greedy BFS'].time_ms:.2f}",
        ),
        (
            "Optimality",
            OPTIMALITY_LABELS["Dijkstra"],
            OPTIMALITY_LABELS["A*"],
            OPTIMALITY_LABELS["Greedy BFS"],
        ),
    ]

    lines = [
        f"Wind Scenario: {scenario.name} | {scenario.description} | W={scenario.wind}",
        header,
        separator,
    ]
    for metric, dijkstra_value, a_star_value, greedy_value in rows:
        lines.append(
            f"{metric:<{metric_width}}"
            f"{dijkstra_value:>{col_width}}"
            f"{a_star_value:>{col_width}}"
            f"{greedy_value:>{col_width}}"
        )
    return "\n".join(lines)


def percentage_reduction(baseline: float, improved: float) -> float:
    """Return percentage reduction from baseline to improved."""

    if baseline == 0.0:
        return 0.0
    return ((baseline - improved) / baseline) * 100.0


def write_analysis(
    scenario_results: Sequence[ScenarioResult], visit_order: Sequence[str]
) -> None:
    """Write the required written analysis to analysis.txt."""

    table_text = "\n\n".join(format_comparison_table(scenario) for scenario in scenario_results)

    average_altitudes = {
        scenario.name: np.mean(
            [node[2] * Z_SCALE_METRES for node in scenario.results["A*"].mission_path]
        )
        for scenario in scenario_results
    }
    max_altitudes = {
        scenario.name: max(node[2] * Z_SCALE_METRES for node in scenario.results["A*"].mission_path)
        for scenario in scenario_results
    }
    wind_totals = {
        scenario.name: sum(
            component["wind_cost"] + component["crosswind_cost"]
            for component in scenario.results["A*"].edge_components
        )
        for scenario in scenario_results
    }
    path_identical_pairs = []
    for left_index, left in enumerate(scenario_results):
        for right in scenario_results[left_index + 1 :]:
            identical = left.results["A*"].mission_path == right.results["A*"].mission_path
            path_identical_pairs.append((left.name, right.name, identical))

    lowest_cost_scenario = min(scenario_results, key=lambda item: item.results["A*"].total_cost)
    highest_cost_scenario = max(scenario_results, key=lambda item: item.results["A*"].total_cost)

    climb_cost_z2_to_z6 = DRONE_MASS_KG * GRAVITY_MPS2 * ((6 - 2) * Z_SCALE_METRES)
    strongest_wind_savings = min(wind_totals.items(), key=lambda item: item[1])
    strongest_wind_penalty = max(wind_totals.items(), key=lambda item: item[1])

    reductions = []
    for scenario in scenario_results:
        reduction = percentage_reduction(
            scenario.results["Dijkstra"].nodes_explored,
            scenario.results["A*"].nodes_explored,
        )
        reductions.append(f"{scenario.name}: {reduction:.2f}% fewer expanded nodes")

    analysis = f"""3D Drone Delivery Route Optimization - Analysis

Visit order used for the multi-stop mission:
{' -> '.join(visit_order)}

Comparison tables:
{table_text}

7.1 Time Complexity
Algorithm            Time Complexity         Explanation
Dijkstra             O((V + E) log V)       V = X*Y*Z nodes, E = up to 26*V edges, heap operations cost O(log V)
A*                   O((V + E) log V)       Worst case matches Dijkstra, but the heuristic prunes large regions in practice
Greedy BFS           O((V + E) log V)       Worst case still explores much of the graph, but typically runs quickly
TSP Greedy NN        O(n^2) + A* per leg    Ordering over n waypoints is quadratic; pathfinding dominates total runtime

7.2 Space Complexity
Dijkstra and A* both require O(V) space for their distance and predecessor structures and O(V) space in the priority queue in the worst case.
For this project, V = {X_MAX * Y_MAX * Z_MAX} nodes, so the full 3D search space remains comfortably feasible in memory.
Greedy BFS also stores predecessor/frontier information bounded by O(V).

7.3 Effect of Wind on Optimal Path
The A* mission cost is lowest in {lowest_cost_scenario.name} ({lowest_cost_scenario.results['A*'].total_cost:.2f} units) and highest in {highest_cost_scenario.name} ({highest_cost_scenario.results['A*'].total_cost:.2f} units).
Average A* mission altitude by scenario (metres): {', '.join(f"{name}={value:.2f}" for name, value in average_altitudes.items())}.
Maximum A* altitude by scenario (metres): {', '.join(f"{name}={value:.2f}" for name, value in max_altitudes.items())}.
Combined wind-plus-crosswind contribution along the A* mission path (cost units): {', '.join(f"{name}={value:.2f}" for name, value in wind_totals.items())}.
Path geometry equality checks across scenarios: {', '.join(f"{left} vs {right}={'same' if same else 'different'}" for left, right, same in path_identical_pairs)}.
These numbers show whether the physically optimal route changes only in total energy, or also in its spatial shape and altitude preference, when the wind field changes.

7.4 Why A* Outperforms Dijkstra
A* uses a physics-aware lower bound built from straight-line distance plus conservative altitude and wind terms to focus the search toward the current goal, while Dijkstra expands outward uniformly without directionality.
Observed node-expansion reductions relative to Dijkstra: {', '.join(reductions)}.
This is especially useful in a 3D grid because the branching factor is up to 26 neighbours per node, so heuristic guidance avoids wasting work in off-route volume.

7.5 Altitude vs Energy Trade-off
Climbing from z=2 to z=6 means a vertical gain of 60 metres, so the gravitational climb cost is:
2.5 * 9.81 * 60 = {climb_cost_z2_to_z6:.1f} cost units.
The strongest observed wind-related saving over an A* mission path was {strongest_wind_savings[1]:.2f} cost units in {strongest_wind_savings[0]}, while the strongest wind-related penalty was {strongest_wind_penalty[1]:.2f} cost units in {strongest_wind_penalty[0]}.
This comparison quantifies when a higher-altitude route can pay off: only when the wind and crosswind terms along that route offset a meaningful portion of the climb energy. Otherwise, staying lower is cheaper because gravitational cost grows directly with altitude gain.
"""

    ANALYSIS_PATH.write_text(analysis, encoding="utf-8")


# ============================================================================
# VALIDATION
# ============================================================================

def run_self_checks(obstacles: ObstacleSet) -> None:
    """Run lightweight internal validation checks before the main experiments."""

    assert obstacles == generate_obstacles(), "Obstacle generation must be deterministic."
    assert tsp_greedy() == ["Depot", "D2", "D3", "D1", "Depot"]

    for node in WAYPOINTS.values():
        assert is_valid(node, obstacles), f"Waypoint should be flyable: {node}"

    flat_components = edge_cost_components((1, 1, 2), (2, 1, 2), (0.0, 0.0, 0.0))
    assert math.isclose(flat_components["base_distance_cost"], 10.0, rel_tol=1e-9)
    assert math.isclose(flat_components["altitude_cost"], 0.0, abs_tol=1e-9)
    assert math.isclose(edge_weight((1, 1, 2), (2, 1, 2), (0.0, 0.0, 0.0)), 10.0, rel_tol=1e-9)

    climb_components = edge_cost_components((1, 1, 2), (1, 1, 3), (0.0, 0.0, 0.0))
    expected_climb_cost = DRONE_MASS_KG * GRAVITY_MPS2 * Z_SCALE_METRES
    assert math.isclose(climb_components["altitude_cost"], expected_climb_cost, rel_tol=1e-9)

    descend_components = edge_cost_components((1, 1, 3), (1, 1, 2), (0.0, 0.0, 0.0))
    expected_descent_cost = REGEN_FACTOR * DRONE_MASS_KG * GRAVITY_MPS2 * Z_SCALE_METRES
    assert math.isclose(
        descend_components["altitude_cost"], expected_descent_cost, rel_tol=1e-9
    )

    extreme_tailwind_weight = edge_weight((1, 1, 3), (2, 2, 2), (1000.0, 1000.0, 1000.0))
    assert extreme_tailwind_weight >= MIN_EDGE_WEIGHT

    interior_node_has_26 = False
    for x in range(1, X_MAX - 1):
        for y in range(1, Y_MAX - 1):
            for z in range(MIN_FLIGHT_Z + 1, MAX_FLIGHT_Z):
                node = (x, y, z)
                if is_valid(node, obstacles) and len(get_neighbours(node, obstacles)) == 26:
                    interior_node_has_26 = True
                    break
            if interior_node_has_26:
                break
        if interior_node_has_26:
            break
    assert interior_node_has_26, "Expected at least one interior node with all 26 neighbours."

    boundary_neighbours = get_neighbours((0, 0, MIN_FLIGHT_Z), obstacles)
    assert all(neighbour[2] >= MIN_FLIGHT_Z for neighbour in boundary_neighbours)
    assert all(neighbour not in obstacles for neighbour in boundary_neighbours)


def validate_scenario_results(scenario_results: Sequence[ScenarioResult]) -> None:
    """Validate the completed mission outputs against project expectations."""

    for scenario in scenario_results:
        dijkstra_result = scenario.results["Dijkstra"]
        a_star_result = scenario.results["A*"]
        greedy_result = scenario.results["Greedy BFS"]

        assert dijkstra_result.mission_path, f"Dijkstra mission path missing for {scenario.name}"
        assert a_star_result.mission_path, f"A* mission path missing for {scenario.name}"
        assert greedy_result.mission_path, f"Greedy BFS mission path missing for {scenario.name}"

        assert math.isclose(
            dijkstra_result.total_cost,
            a_star_result.total_cost,
            rel_tol=1e-9,
            abs_tol=1e-9,
        ), f"A* and Dijkstra costs should match for {scenario.name}"

        assert a_star_result.nodes_explored <= dijkstra_result.nodes_explored, (
            f"A* should explore no more nodes than Dijkstra for {scenario.name}"
        )


# ============================================================================
# MAIN
# ============================================================================

def main() -> None:
    """Run all algorithms for all wind scenarios and generate deliverables."""

    obstacles = generate_obstacles()
    run_self_checks(obstacles)

    visit_order = tsp_greedy()
    algorithms: List[Tuple[str, AlgorithmFunction]] = [
        ("Dijkstra", dijkstra),
        ("A*", a_star),
        ("Greedy BFS", greedy_bfs),
    ]

    scenario_results: List[ScenarioResult] = []
    for scenario_name, wind, description in WIND_SCENARIOS:
        results: Dict[str, MissionResult] = {}
        for algorithm_name, algorithm in algorithms:
            results[algorithm_name] = run_mission(
                algorithm_name=algorithm_name,
                algorithm=algorithm,
                visit_order=visit_order,
                obstacles=obstacles,
                wind=wind,
            )

        scenario = ScenarioResult(
            name=scenario_name,
            wind=wind,
            description=description,
            results=results,
        )
        scenario_results.append(scenario)
        print(format_comparison_table(scenario))
        print()

    validate_scenario_results(scenario_results)

    create_route_plot(scenario_results, obstacles)
    create_cost_breakdown_plot(scenario_results)
    create_algorithm_comparison_plot(scenario_results)
    create_altitude_profile_plot(scenario_results)
    write_analysis(scenario_results, visit_order)

    generated_files = [
        ROUTE_PLOT_PATH,
        COST_BREAKDOWN_PATH,
        ALGO_COMPARISON_PATH,
        ALTITUDE_PROFILE_PATH,
        ANALYSIS_PATH,
    ]
    for output_path in generated_files:
        assert output_path.exists(), f"Expected output file missing: {output_path}"

    print("Generated files:")
    for output_path in generated_files:
        print(f" - {output_path.resolve()}")


if __name__ == "__main__":
    main()
