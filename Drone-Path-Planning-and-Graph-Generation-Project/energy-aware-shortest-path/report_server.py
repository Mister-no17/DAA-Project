import json
import math
import os
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

REPORTS_ROOT = Path(__file__).resolve().parent / "reports"

DIRECTION_VECTORS = {
    "N": (-1.0, 0.0),
    "NE": (-math.sqrt(0.5), math.sqrt(0.5)),
    "E": (0.0, 1.0),
    "SE": (math.sqrt(0.5), math.sqrt(0.5)),
    "S": (1.0, 0.0),
    "SW": (math.sqrt(0.5), -math.sqrt(0.5)),
    "W": (0.0, -1.0),
    "NW": (-math.sqrt(0.5), -math.sqrt(0.5)),
}


def ensure_reports_root() -> None:
    REPORTS_ROOT.mkdir(parents=True, exist_ok=True)


def compute_grid_stats(grid: List[List[float]], blocked: List[str]) -> Tuple[int, int]:
    if not grid:
        return 0, 0

    rows = len(grid)
    cols = len(grid[0])
    blocked_set = set(blocked)
    vertices = 0
    edges = 0

    for row in range(rows):
        for col in range(cols):
            key = f"{row},{col}"
            if key in blocked_set:
                continue

            vertices += 1
            if row + 1 < rows and f"{row + 1},{col}" not in blocked_set:
                edges += 1
            if col + 1 < cols and f"{row},{col + 1}" not in blocked_set:
                edges += 1

    return vertices, edges


def estimate_ops(algorithm_name: str, vertices: int, edges: int) -> float:
    if vertices <= 1 or edges <= 0:
        return 0.0

    return (vertices + edges) * math.log2(vertices)


def compute_edge_components(
    path: List[List[int]],
    altitude_grid: List[List[float]],
    environment: Dict[str, float],
) -> Dict[str, float]:
    if not path or len(path) < 2:
        return {
            "distance": 0.0,
            "wind": 0.0,
            "gravity": 0.0,
            "turning": 0.0,
        }

    wind_direction = environment.get("windDirection", "E")
    wind_strength = float(environment.get("windStrength", 0.0))
    dynamic_wind = bool(environment.get("dynamicWindEnabled", False))
    wind_shift_step = int(environment.get("windShiftStep", 0) or 0)
    wind_direction_after = environment.get("windDirectionAfterShift", wind_direction)
    wind_strength_after = float(environment.get("windStrengthAfterShift", wind_strength))
    wind_shifts = environment.get("windShifts", []) if isinstance(environment.get("windShifts", []), list) else []
    gust_regions = environment.get("gustRegions", []) if isinstance(environment.get("gustRegions", []), list) else []
    distance_coefficient = float(environment.get("distanceCoefficient", 1.0))
    wind_coefficient = float(environment.get("windCoefficient", 1.0))
    turning_coefficient = float(environment.get("turningCoefficient", 0.0))
    mass = float(environment.get("mass", environment.get("altitudeFactor", 1.0)))
    gravity = float(environment.get("gravity", 9.81))

    distance_total = 0.0
    wind_total = 0.0
    gravity_total = 0.0
    turning_total = 0.0

    def turning_angle(prev_vec: Tuple[float, float], next_vec: Tuple[float, float]) -> float:
        prev_norm = math.hypot(prev_vec[0], prev_vec[1])
        next_norm = math.hypot(next_vec[0], next_vec[1])
        if prev_norm == 0 or next_norm == 0:
            return 0.0
        dot = (prev_vec[0] * next_vec[0] + prev_vec[1] * next_vec[1]) / (prev_norm * next_norm)
        clamped = max(-1.0, min(1.0, dot))
        return math.acos(clamped)

    def resolve_wind_state(step_index: int, from_node: List[int], to_node: List[int]) -> Tuple[str, float]:
        active_direction = wind_direction
        active_strength = wind_strength

        if wind_shifts:
            ordered = sorted(wind_shifts, key=lambda event: int(event.get("step", 0)))
            for shift in ordered:
                shift_step = int(shift.get("step", 0) or 0)
                transition = int(shift.get("transitionSteps", 0) or 0)
                transition_start = max(0, shift_step - transition)

                if transition > 0 and transition_start <= step_index < shift_step:
                    progress = (step_index - transition_start + 1) / transition
                    progress = max(0.0, min(1.0, progress))
                    target_strength = float(shift.get("strength", active_strength))
                    active_strength = active_strength + (target_strength - active_strength) * progress
                    active_direction = shift.get("direction", active_direction)
                    break

                if step_index >= shift_step:
                    active_direction = shift.get("direction", active_direction)
                    active_strength = float(shift.get("strength", active_strength))
                else:
                    break
        elif dynamic_wind and step_index >= wind_shift_step:
            active_direction = wind_direction_after
            active_strength = wind_strength_after

        for gust in gust_regions:
            start_step = int(gust.get("startStep", 0) or 0)
            end_step = int(gust.get("endStep", start_step) or start_step)
            if step_index < start_step or step_index > end_step:
                continue
            rows = gust.get("rows", [0, -1])
            cols = gust.get("cols", [0, -1])
            row_min, row_max = int(rows[0]), int(rows[1])
            col_min, col_max = int(cols[0]), int(cols[1])
            if not (row_min <= from_node[0] <= row_max and col_min <= from_node[1] <= col_max) and not (
                row_min <= to_node[0] <= row_max and col_min <= to_node[1] <= col_max
            ):
                continue
            if gust.get("direction"):
                active_direction = gust.get("direction")
            strength_multiplier = float(gust.get("strengthMultiplier", 1.0))
            strength_offset = float(gust.get("strengthOffset", 0.0))
            active_strength = max(0.0, active_strength * strength_multiplier + strength_offset)

        return active_direction, active_strength

    for idx in range(len(path) - 1):
        from_node = path[idx]
        to_node = path[idx + 1]
        active_direction, active_strength = resolve_wind_state(idx, from_node, to_node)

        wind_vector = DIRECTION_VECTORS.get(active_direction, DIRECTION_VECTORS["E"])

        move_row = to_node[0] - from_node[0]
        move_col = to_node[1] - from_node[1]
        norm = math.hypot(move_row, move_col) or 1.0
        unit_row = move_row / norm
        unit_col = move_col / norm
        wind_norm = math.hypot(wind_vector[0], wind_vector[1]) or 1.0
        unit_wind_row = wind_vector[0] / wind_norm
        unit_wind_col = wind_vector[1] / wind_norm
        dot = unit_row * unit_wind_row + unit_col * unit_wind_col
        clamped = max(-1.0, min(1.0, dot))
        theta = math.acos(clamped)

        distance = math.hypot(move_row, move_col)
        wind_effect = wind_coefficient * active_strength * (1 - math.cos(theta))
        current_alt = altitude_grid[from_node[0]][from_node[1]]
        next_alt = altitude_grid[to_node[0]][to_node[1]]
        climb = max(0.0, next_alt - current_alt)
        gravity_cost = mass * gravity * climb

        if idx > 0:
            prev_node = path[idx - 1]
            prev_vec = (from_node[0] - prev_node[0], from_node[1] - prev_node[1])
            next_vec = (to_node[0] - from_node[0], to_node[1] - from_node[1])
            turn_angle = turning_angle(prev_vec, next_vec)
        else:
            turn_angle = 0.0
        # Quadratic turn penalty mirrors curvature-based yaw effort used in the solver.
        turning_cost = turning_coefficient * (turn_angle * turn_angle)

        distance_total += distance_coefficient * distance
        wind_total += wind_effect
        gravity_total += gravity_cost
        turning_total += turning_cost

    return {
        "distance": distance_total,
        "wind": wind_total,
        "gravity": gravity_total,
        "turning": turning_total,
    }


def plot_cost_comparison(metrics: Dict[str, Dict[str, float]], output_path: Path) -> None:
    names = list(metrics.keys())
    costs = [metrics[name]["totalEnergyCost"] for name in names]
    colors = [
        {
            "Standard Dijkstra": "#f08c00",
            "Energy-Aware A*": "#0d8cb6",
            "Energy-Aware Theta*": "#7a4ae0",
        }.get(name, "#64748b")
        for name in names
    ]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    bars = ax.bar(names, costs, color=colors)
    ax.set_title("Total Energy Cost Comparison")
    ax.set_ylabel("Energy Cost")
    ax.bar_label(bars, fmt="%.2f")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_nodes_explored(metrics: Dict[str, Dict[str, float]], output_path: Path) -> None:
    names = list(metrics.keys())
    nodes = [metrics[name]["expandedNodes"] for name in names]
    colors = [
        {
            "Standard Dijkstra": "#f08c00",
            "Energy-Aware A*": "#0d8cb6",
            "Energy-Aware Theta*": "#7a4ae0",
        }.get(name, "#64748b")
        for name in names
    ]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    bars = ax.bar(names, nodes, color=colors)
    ax.set_title("Expanded Nodes Comparison")
    ax.set_ylabel("Expanded Nodes")
    ax.bar_label(bars)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_altitude_profile(paths: Dict[str, List[List[int]]], altitude_grid: List[List[float]], output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(9, 4.5))

    colors = {
        "Standard Dijkstra": "red",
        "Energy-Aware A*": "blue",
        "Energy-Aware Theta*": "#7a4ae0",
    }

    for name, path in paths.items():
        if not path:
            continue
        altitudes = [altitude_grid[row][col] for row, col in path]
        ax.plot(range(len(altitudes)), altitudes, label=name, color=colors.get(name, "#333"))

    ax.set_title("Altitude Profile by Algorithm")
    ax.set_xlabel("Path Step")
    ax.set_ylabel("Altitude")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_3d_route_visualization(
    paths: Dict[str, List[List[int]]],
    altitude_grid: List[List[float]],
    blocked: List[str],
    output_path: Path,
) -> None:
    if not altitude_grid:
        return

    rows = len(altitude_grid)
    cols = len(altitude_grid[0])
    grid = np.array(altitude_grid, dtype=float)
    x_coords = np.arange(cols)
    y_coords = np.arange(rows)
    mesh_x, mesh_y = np.meshgrid(x_coords, y_coords)

    fig = plt.figure(figsize=(10, 7.5))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot_surface(
        mesh_x,
        mesh_y,
        grid,
        cmap="terrain",
        alpha=0.72,
        linewidth=0.0,
        antialiased=True,
    )

    blocked_coords = [tuple(map(int, cell.split(","))) for cell in blocked]
    if blocked_coords:
        for row, col in blocked_coords:
            height = max(grid[row, col], 0.6)
            ax.bar3d(
                col - 0.4,
                row - 0.4,
                0.0,
                0.8,
                0.8,
                height,
                color="#1e1e1e",
                alpha=0.9,
                shade=True,
            )

    styles = {
        "Standard Dijkstra": {"color": "red", "linestyle": "--", "linewidth": 2.2},
        "Energy-Aware A*": {"color": "#1e6cf2", "linestyle": "-", "linewidth": 2.6},
        "Energy-Aware Theta*": {"color": "#7a4ae0", "linestyle": "-.", "linewidth": 2.4},
    }

    for name, path in paths.items():
        if len(path) < 2:
            continue

        path_x = [coord[1] for coord in path]
        path_y = [coord[0] for coord in path]
        path_z = [grid[coord[0], coord[1]] + 0.2 for coord in path]
        style = styles.get(name, {"color": "#333", "linestyle": "-", "linewidth": 2.0})
        ax.plot(path_x, path_y, path_z, label=name, **style)

    start = next((path[0] for path in paths.values() if path), None)
    end = next((path[-1] for path in paths.values() if path), None)

    if start:
        ax.scatter(
            [start[1]],
            [start[0]],
            [grid[start[0], start[1]] + 0.4],
            color="#2a9d3f",
            marker="*",
            s=180,
            label="Start",
        )

    if end:
        ax.scatter(
            [end[1]],
            [end[0]],
            [grid[end[0], end[1]] + 0.4],
            color="#d1a317",
            marker="D",
            s=120,
            label="End",
        )

    ax.set_title("3D Route Visualization")
    ax.set_xlabel("X (Grid Column)")
    ax.set_ylabel("Y (Grid Row)")
    ax.set_zlabel("Altitude")
    ax.view_init(elev=35, azim=-50)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_edge_cost_breakdown(components: Dict[str, Dict[str, float]], output_path: Path) -> None:
    names = list(components.keys())
    distance_vals = [components[name]["distance"] for name in names]
    wind_vals = [components[name]["wind"] for name in names]
    gravity_vals = [components[name]["gravity"] for name in names]
    turning_vals = [components[name]["turning"] for name in names]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.bar(names, distance_vals, label="Distance", color="#f6b04b")
    ax.bar(names, wind_vals, bottom=distance_vals, label="Wind", color="#4fb0d1")
    stacked = [distance_vals[i] + wind_vals[i] for i in range(len(names))]
    ax.bar(names, gravity_vals, bottom=stacked, label="Gravity", color="#7a4ae0")
    stacked = [stacked[i] + gravity_vals[i] for i in range(len(names))]
    ax.bar(names, turning_vals, bottom=stacked, label="Turning", color="#2f8f6b")

    ax.set_title("Edge Cost Breakdown")
    ax.set_ylabel("Cost")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_runtime_comparison(metrics: Dict[str, Dict[str, float]], output_path: Path) -> None:
    names = list(metrics.keys())
    runtimes = [metrics[name]["executionTimeMs"] for name in names]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    bars = ax.bar(names, runtimes, color=["#f08c00", "#0d8cb6", "#7a4ae0"])
    ax.set_title("Runtime Comparison (ms)")
    ax.set_ylabel("Milliseconds")
    ax.bar_label(bars, fmt="%.2f")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_curvature_metrics(metrics: Dict[str, Dict[str, float]], output_path: Path) -> None:
    names = list(metrics.keys())
    turning_energy = [metrics[name]["turningEnergy"] for name in names]
    turning_count = [metrics[name]["turningCount"] for name in names]
    heading_change = [metrics[name]["averageHeadingChange"] for name in names]

    fig, ax = plt.subplots(figsize=(9, 4.6))
    x = np.arange(len(names))
    ax.bar(x - 0.25, turning_energy, width=0.25, label="Turning energy", color="#2f8f6b")
    ax.bar(x, turning_count, width=0.25, label="Turn count", color="#4fb0d1")
    ax.bar(x + 0.25, heading_change, width=0.25, label="Avg heading change (rad)", color="#7a4ae0")
    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_title("Curvature & Turning Metrics")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_replanning_timeline(dynamic_env: Dict[str, object], output_path: Path) -> None:
    timeline = dynamic_env.get("timeline", []) if isinstance(dynamic_env, dict) else []
    change_steps = set(dynamic_env.get("changeSteps", []) if isinstance(dynamic_env, dict) else [])

    if not timeline:
        return

    steps = [state.get("step", 0) for state in timeline]
    strengths = [state.get("strength", 0.0) for state in timeline]

    fig, ax = plt.subplots(figsize=(9, 3.8))
    ax.plot(steps, strengths, color="#0d8cb6", linewidth=2.0, label="Wind strength")

    for step in change_steps:
        ax.axvline(step, color="#f08c00", linestyle="--", linewidth=1.2, alpha=0.8)

    ax.set_title("Dynamic Wind Timeline")
    ax.set_xlabel("Step")
    ax.set_ylabel("Wind Strength")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_wind_field(environment: Dict[str, object], altitude_grid: List[List[float]], output_path: Path) -> None:
    rows = len(altitude_grid) if altitude_grid else 8
    cols = len(altitude_grid[0]) if altitude_grid else 8
    base_direction = environment.get("windDirection", "E") if isinstance(environment, dict) else "E"
    base_strength = float(environment.get("windStrength", 1.0)) if isinstance(environment, dict) else 1.0

    vector = DIRECTION_VECTORS.get(base_direction, DIRECTION_VECTORS["E"])
    u = np.full((rows, cols), vector[1])
    v = np.full((rows, cols), -vector[0])

    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    ax.quiver(u, v, angles="xy", scale_units="xy", scale=1.8 / max(base_strength, 0.4), color="#0d8cb6")
    ax.set_title("Wind Vector Field (Base)")
    ax.set_xticks([])
    ax.set_yticks([])
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_route_heatmap(paths: Dict[str, List[List[int]]], altitude_grid: List[List[float]], output_path: Path) -> None:
    if not altitude_grid:
        return

    rows = len(altitude_grid)
    cols = len(altitude_grid[0])
    heat = np.zeros((rows, cols), dtype=float)

    for path in paths.values():
        for row, col in path:
            heat[row, col] += 1

    fig, ax = plt.subplots(figsize=(6.5, 5.5))
    im = ax.imshow(heat, cmap="magma")
    ax.set_title("Route Density Heatmap")
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def build_metrics_payload(algorithms: List[Dict[str, float]], vertices: int, edges: int) -> Dict[str, Dict[str, float]]:
    def safe_float(value: object) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return 0.0
        return number if math.isfinite(number) else 0.0

    metrics: Dict[str, Dict[str, float]] = {}
    for entry in algorithms:
        name = entry["name"]
        is_placeholder = bool(entry.get("isPlaceholder", False))
        metrics[name] = {
            "executionTimeMs": safe_float(entry.get("executionTimeMs")),
            "expandedNodes": safe_float(entry.get("expandedNodes")),
            "totalEnergyCost": safe_float(entry.get("totalEnergyCost")),
            "totalDistance": safe_float(entry.get("totalDistance")),
            "windEnergy": safe_float(entry.get("windEnergy")),
            "gravityEnergy": safe_float(entry.get("gravityEnergy")),
            "turningEnergy": safe_float(entry.get("turningEnergy")),
            "turningCount": safe_float(entry.get("turningCount")),
            "averageHeadingChange": safe_float(entry.get("averageHeadingChange")),
            "smoothnessScore": safe_float(entry.get("smoothnessScore")),
            "heuristicEvaluations": safe_float(entry.get("heuristicEvaluations")),
            "losChecks": safe_float(entry.get("losChecks")),
            "losSuccess": safe_float(entry.get("losSuccess")),
            "replanCount": safe_float(entry.get("replanCount")),
            "environmentShiftCount": safe_float(entry.get("environmentShiftCount")),
            "routeStability": safe_float(entry.get("routeStability")),
            "energyFluctuation": safe_float(entry.get("energyFluctuation")),
            "adaptationOverheadMs": safe_float(entry.get("adaptationOverheadMs")),
            "objectiveCost": safe_float(entry.get("objectiveCost")),
            "estimatedOperations": estimate_ops(name, vertices, edges),
            "theoreticalComplexity": "N/A" if is_placeholder else "O((V + E) log V)",
        }
    return metrics


def handle_report(payload: Dict[str, object]) -> Dict[str, str]:
    ensure_reports_root()

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    run_dir = REPORTS_ROOT / f"run_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)

    grid = payload.get("altitudeGrid", [])
    blocked = payload.get("obstacles", [])
    vertices, edges = compute_grid_stats(grid, blocked)
    algorithms = payload.get("algorithms", [])

    summary = {
        "scenarioName": payload.get("scenarioName"),
        "timestamp": payload.get("timestampIso"),
        "environment": payload.get("environment"),
        "grid": {
            "rows": payload.get("grid", {}).get("rows"),
            "cols": payload.get("grid", {}).get("cols"),
            "vertices": vertices,
            "edges": edges,
        },
        "obstacleCount": len(blocked),
        "activeAlgorithms": [entry.get("name") for entry in algorithms],
    }

    metrics = build_metrics_payload(algorithms, vertices, edges)

    path_data = {
        "paths": payload.get("paths"),
        "blockedCells": blocked,
        "altitudeGrid": grid,
        "start": payload.get("start"),
        "end": payload.get("end"),
    }

    with open(run_dir / "summary.json", "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    with open(run_dir / "metrics.json", "w", encoding="utf-8") as handle:
        json.dump(metrics, handle, indent=2)

    with open(run_dir / "path_data.json", "w", encoding="utf-8") as handle:
        json.dump(path_data, handle, indent=2)

    paths = payload.get("paths", {})
    path_map = {
        "Standard Dijkstra": paths.get("standard", []),
        "Energy-Aware A*": paths.get("energyAStar", []),
        "Energy-Aware Theta*": paths.get("thetaStar", []),
    }

    component_totals = {}
    environment = payload.get("environment", {})
    dynamic_environment = payload.get("dynamicEnvironment", {})
    for name, path in path_map.items():
        component_totals[name] = compute_edge_components(path, grid, environment)

    plot_cost_comparison(metrics, run_dir / "cost_comparison.png")
    plot_nodes_explored(metrics, run_dir / "nodes_explored.png")
    plot_altitude_profile(path_map, grid, run_dir / "altitude_profile.png")
    plot_3d_route_visualization(path_map, grid, blocked, run_dir / "3d_route_visualization.png")
    plot_edge_cost_breakdown(component_totals, run_dir / "edge_cost_breakdown.png")
    plot_runtime_comparison(metrics, run_dir / "runtime_comparison.png")
    plot_curvature_metrics(metrics, run_dir / "curvature_metrics.png")
    plot_replanning_timeline(dynamic_environment, run_dir / "replanning_timeline.png")
    plot_wind_field(environment, grid, run_dir / "wind_vector_field.png")
    plot_route_heatmap(path_map, grid, run_dir / "route_heatmap.png")

    return {"folder": str(run_dir)}


class ReportHandler(BaseHTTPRequestHandler):
    def do_OPTIONS(self) -> None:  # noqa: N802
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_POST(self) -> None:  # noqa: N802
        if self.path != "/report":
            self.send_response(404)
            self.end_headers()
            return

        content_length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(content_length)
        try:
            payload = json.loads(body.decode("utf-8"))
        except json.JSONDecodeError:
            self.send_response(400)
            self.end_headers()
            return

        try:
            result = handle_report(payload)
        except Exception as exc:  # noqa: BLE001
            self.send_response(500)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(str(exc).encode("utf-8"))
            return

        response_body = json.dumps(result).encode("utf-8")
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(response_body)))
        self.end_headers()
        self.wfile.write(response_body)


def main() -> None:
    server = ThreadingHTTPServer(("localhost", 8001), ReportHandler)
    print("Report server running at http://localhost:8001")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
