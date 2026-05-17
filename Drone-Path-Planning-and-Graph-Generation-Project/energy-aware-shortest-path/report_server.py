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

    if algorithm_name == "Bellman-Ford (DP)":
        return vertices * edges

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
            "altitude": 0.0,
        }

    wind_direction = environment.get("windDirection", "E")
    wind_strength = float(environment.get("windStrength", 0.0))
    dynamic_wind = bool(environment.get("dynamicWindEnabled", False))
    wind_shift_step = int(environment.get("windShiftStep", 0) or 0)
    wind_direction_after = environment.get("windDirectionAfterShift", wind_direction)
    wind_strength_after = float(environment.get("windStrengthAfterShift", wind_strength))
    altitude_factor = float(environment.get("altitudeFactor", 0.0))
    downhill_factor = float(environment.get("downhillFactor", 0.35))

    distance_total = 0.0
    wind_total = 0.0
    altitude_total = 0.0

    for idx in range(len(path) - 1):
        from_node = path[idx]
        to_node = path[idx + 1]
        if dynamic_wind and idx >= wind_shift_step:
            active_direction = wind_direction_after
            active_strength = wind_strength_after
        else:
            active_direction = wind_direction
            active_strength = wind_strength

        wind_vector = DIRECTION_VECTORS.get(active_direction, DIRECTION_VECTORS["E"])

        move_row = to_node[0] - from_node[0]
        move_col = to_node[1] - from_node[1]
        norm = math.hypot(move_row, move_col) or 1.0
        unit_row = move_row / norm
        unit_col = move_col / norm
        dot = unit_row * wind_vector[0] + unit_col * wind_vector[1]

        distance = math.hypot(move_row, move_col)
        wind_effect = active_strength * (1 - dot)
        current_alt = altitude_grid[from_node[0]][from_node[1]]
        next_alt = altitude_grid[to_node[0]][to_node[1]]
        climb = max(0.0, next_alt - current_alt)
        descent = max(0.0, current_alt - next_alt)
        altitude_cost = altitude_factor * (climb + downhill_factor * descent)

        distance_total += distance
        wind_total += wind_effect
        altitude_total += altitude_cost

    return {
        "distance": distance_total,
        "wind": wind_total,
        "altitude": altitude_total,
    }


def plot_cost_comparison(metrics: Dict[str, Dict[str, float]], output_path: Path) -> None:
    names = list(metrics.keys())
    costs = [metrics[name]["totalEnergyCost"] for name in names]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    bars = ax.bar(names, costs, color=["#f08c00", "#0d8cb6", "#7a4ae0"])
    ax.set_title("Total Energy Cost Comparison")
    ax.set_ylabel("Energy Cost")
    ax.bar_label(bars, fmt="%.2f")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_nodes_explored(metrics: Dict[str, Dict[str, float]], output_path: Path) -> None:
    names = list(metrics.keys())
    nodes = [metrics[name]["expandedNodes"] for name in names]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    bars = ax.bar(names, nodes, color=["#f08c00", "#0d8cb6", "#7a4ae0"])
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
        "Modified Dijkstra": "blue",
        "Bellman-Ford (DP)": "purple",
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
        "Modified Dijkstra": {"color": "#1e6cf2", "linestyle": "-", "linewidth": 2.6},
        "Bellman-Ford (DP)": {"color": "#7a4ae0", "linestyle": ":", "linewidth": 2.2},
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
    altitude_vals = [components[name]["altitude"] for name in names]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.bar(names, distance_vals, label="Distance", color="#f6b04b")
    ax.bar(names, wind_vals, bottom=distance_vals, label="Wind", color="#4fb0d1")
    stacked = [distance_vals[i] + wind_vals[i] for i in range(len(names))]
    ax.bar(names, altitude_vals, bottom=stacked, label="Altitude", color="#9165e6")

    ax.set_title("Edge Cost Breakdown")
    ax.set_ylabel("Cost")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def build_metrics_payload(algorithms: List[Dict[str, float]], vertices: int, edges: int) -> Dict[str, Dict[str, float]]:
    metrics: Dict[str, Dict[str, float]] = {}
    for entry in algorithms:
        name = entry["name"]
        metrics[name] = {
            "executionTimeMs": entry["executionTimeMs"],
            "expandedNodes": entry["expandedNodes"],
            "totalEnergyCost": entry["totalEnergyCost"],
            "totalDistance": entry["totalDistance"],
            "windPenalty": entry["windPenalty"],
            "altitudePenalty": entry["altitudePenalty"],
            "objectiveCost": entry["objectiveCost"],
            "estimatedOperations": estimate_ops(name, vertices, edges),
            "theoreticalComplexity": "O(VE)" if name == "Bellman-Ford (DP)" else "O((V + E) log V)",
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
        "Modified Dijkstra": paths.get("modified", []),
        "Bellman-Ford (DP)": paths.get("bellmanFord", []),
    }

    component_totals = {}
    environment = payload.get("environment", {})
    for name, path in path_map.items():
        component_totals[name] = compute_edge_components(path, grid, environment)

    plot_cost_comparison(metrics, run_dir / "cost_comparison.png")
    plot_nodes_explored(metrics, run_dir / "nodes_explored.png")
    plot_altitude_profile(path_map, grid, run_dir / "altitude_profile.png")
    plot_3d_route_visualization(path_map, grid, blocked, run_dir / "3d_route_visualization.png")
    plot_edge_cost_breakdown(component_totals, run_dir / "edge_cost_breakdown.png")

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
