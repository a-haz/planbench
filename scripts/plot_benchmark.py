#!/usr/bin/env python3
"""Draw path and metric figures from planbench benchmark files."""

import argparse
import csv
import os
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


COLORS = {
    "astar": "#0072B2",
    "rrt_star": "#D55E00",
    "state_lattice": "#009E73",
}
LABELS = {
    "astar": "A*",
    "rrt_star": "RRT*",
    "state_lattice": "State lattice",
}
PLANNERS = ["astar", "rrt_star", "state_lattice"]


def load_metrics(path):
    summary = []
    curve = []
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            if row["table"] == "summary":
                summary.append(row)
            elif row["table"] == "rrt_budget":
                curve.append(row)
    if not summary or not curve:
        raise SystemExit(f"{path} is missing summary or rrt_budget rows")
    return summary, curve


def as_float(text):
    if text is None or text == "":
        return None
    return float(text)


def load_paths(path):
    meta = {}
    cells = []
    series = defaultdict(list)
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            if row["kind"] == "meta":
                meta[row["series"]] = row["a"]
            elif row["kind"] == "cell":
                cells.append((int(row["a"]), int(row["b"])))
            elif row["kind"] == "path":
                series[row["series"]].append((float(row["a"]), float(row["b"])))
    required = ["width", "height", "resolution", "density", "seed", "start_x", "start_y", "goal_x", "goal_y"]
    missing = [key for key in required if key not in meta]
    if missing:
        raise SystemExit(f"{path} is missing {', '.join(missing)}")
    return meta, cells, series


def plot_example(meta, cells, series, destination):
    width = int(float(meta["width"]))
    height = int(float(meta["height"]))
    resolution = float(meta["resolution"])
    image = np.zeros((height, width))
    for ix, iy in cells:
        image[iy, ix] = 1.0

    fig, ax = plt.subplots(figsize=(7.2, 7.0))
    ax.imshow(
        image,
        origin="lower",
        cmap="gray_r",
        vmin=0.0,
        vmax=1.0,
        extent=(0.0, width * resolution, 0.0, height * resolution),
        interpolation="nearest",
    )
    for name in ("state_lattice", "rrt_star", "astar"):
        points = series.get(name, [])
        if len(points) < 2:
            continue
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        ax.plot(xs, ys, color=COLORS[name], linewidth=2.0, label=LABELS[name], solid_capstyle="round")
    start = (float(meta["start_x"]), float(meta["start_y"]))
    goal = (float(meta["goal_x"]), float(meta["goal_y"]))
    ax.scatter([start[0]], [start[1]], s=46, c="#111111", marker="o", zorder=5, label="Start")
    ax.scatter([goal[0]], [goal[1]], s=70, c="#111111", marker="X", zorder=5, label="Goal")
    ax.set_xlim(0.0, width * resolution)
    ax.set_ylim(0.0, height * resolution)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(
        f"Synthetic map, density {float(meta['density']):.2f}, seed {int(float(meta['seed']))}"
    )
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=5, frameon=False)
    fig.tight_layout()
    fig.savefig(destination, dpi=160, bbox_inches="tight")
    plt.close(fig)


def grouped_values(summary, field):
    densities = []
    for row in summary:
        if row["density"] not in densities:
            densities.append(row["density"])
    values = {name: [] for name in PLANNERS}
    for density in densities:
        for name in PLANNERS:
            match = next(row for row in summary if row["density"] == density and row["planner"] == name)
            values[name].append(as_float(match[field]))
    return densities, values


def plot_comparisons(summary, destination):
    densities, lengths = grouped_values(summary, "mean_path_length_m")
    _, runtimes = grouped_values(summary, "mean_runtime_ms")
    positions = np.arange(len(densities))
    width = 0.24

    fig, axes = plt.subplots(1, 2, figsize=(10.4, 4.3))
    for axis, series, ylabel, title in (
        (axes[0], lengths, "Mean path length (m)", "Path length"),
        (axes[1], runtimes, "Mean runtime (ms)", "Runtime"),
    ):
        for index, name in enumerate(PLANNERS):
            raw = series[name]
            heights = [np.nan if value is None else value for value in raw]
            axis.bar(
                positions + (index - 1) * width,
                heights,
                width,
                color=COLORS[name],
                label=LABELS[name],
            )
        axis.set_xticks(positions)
        axis.set_xticklabels([f"{float(item):.2f}" for item in densities])
        axis.set_xlabel("Obstacle density")
        axis.set_ylabel(ylabel)
        axis.set_title(title)
        axis.legend(frameon=False)
    axes[1].set_yscale("log")
    axes[1].set_ylabel("Mean runtime (ms, log scale)")
    fig.tight_layout()
    fig.savefig(destination, dpi=160, bbox_inches="tight")
    plt.close(fig)


def plot_rrt(curve, destination):
    by_density = defaultdict(list)
    for row in curve:
        by_density[row["density"]].append(row)
    fig, axes = plt.subplots(1, 2, figsize=(10.4, 4.3))
    for density, rows in by_density.items():
        rows = sorted(rows, key=lambda item: int(item["iterations"]))
        iterations = [int(row["iterations"]) for row in rows]
        lengths = [np.nan if as_float(row["mean_path_length_m"]) is None else as_float(row["mean_path_length_m"]) for row in rows]
        rates = [float(row["success_rate"]) for row in rows]
        axes[0].plot(iterations, lengths, marker="o", linewidth=1.8, label=f"density {float(density):.2f}")
        axes[1].plot(iterations, rates, marker="o", linewidth=1.8, label=f"density {float(density):.2f}")
    axes[0].set_xlabel("Iteration budget")
    axes[0].set_ylabel("Mean path length (m)")
    axes[0].set_title("RRT* path length")
    axes[0].legend(frameon=False)
    axes[1].set_xlabel("Iteration budget")
    axes[1].set_ylabel("Success rate")
    axes[1].set_ylim(-0.05, 1.05)
    axes[1].set_title("RRT* success rate")
    axes[1].legend(frameon=False)
    fig.tight_layout()
    fig.savefig(destination, dpi=160, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Plot planbench benchmark results.")
    parser.add_argument("--csv", default="docs/benchmark.csv")
    parser.add_argument("--paths", default="docs/example_paths.csv")
    parser.add_argument("--out-dir", default="docs")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    summary, curve = load_metrics(args.csv)
    meta, cells, series = load_paths(args.paths)
    plot_example(meta, cells, series, f"{args.out_dir}/example_paths.png")
    plot_comparisons(summary, f"{args.out_dir}/length_runtime.png")
    plot_rrt(curve, f"{args.out_dir}/rrt_length_vs_iterations.png")


if __name__ == "__main__":
    main()
