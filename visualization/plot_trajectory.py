"""Plot the A* path and dynamically feasible UAV trajectory in 3D."""
from pathlib import Path
import argparse
import csv

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def load_csv(path):
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    return {key: np.array([float(row[key]) for row in rows]) for key in rows[0]}


def box_faces(row):
    x0, y0, z0, x1, y1, z1 = (
        row[k] for k in ("min_x", "min_y", "min_z", "max_x", "max_y", "max_z")
    )
    p = [
        (x0, y0, z0),
        (x1, y0, z0),
        (x1, y1, z0),
        (x0, y1, z0),
        (x0, y0, z1),
        (x1, y0, z1),
        (x1, y1, z1),
        (x0, y1, z1),
    ]
    return [
        [p[i] for i in face]
        for face in (
            (0, 1, 2, 3),
            (4, 5, 6, 7),
            (0, 1, 5, 4),
            (2, 3, 7, 6),
            (1, 2, 6, 5),
            (3, 0, 4, 7),
        )
    ]


def draw_scene(ax, output):
    raw = load_csv(output / "astar_path.csv")
    smooth = load_csv(output / "smoothed_path.csv")
    trajectory = load_csv(output / "trajectory.csv")
    obstacles = load_csv(output / "obstacles.csv")
    for i in range(len(obstacles["min_x"])):
        row = {k: v[i] for k, v in obstacles.items()}
        ax.add_collection3d(
            Poly3DCollection(
                box_faces(row), alpha=0.28, facecolor="#d62728", edgecolor="#8c1d18"
            )
        )
    ax.plot(raw["x"], raw["y"], raw["z"], ":", color="0.55", label="3D A* voxels")
    ax.plot(
        smooth["x"],
        smooth["y"],
        smooth["z"],
        "--",
        color="#ff7f0e",
        label="collision-free shortcut",
    )
    ax.plot(
        trajectory["x"],
        trajectory["y"],
        trajectory["z"],
        color="#1f77b4",
        lw=2.2,
        label="kinematic trajectory",
    )
    ax.scatter(*[trajectory[k][0] for k in "xyz"], s=65, c="green", label="start")
    ax.scatter(
        *[trajectory[k][-1] for k in "xyz"], s=80, c="purple", marker="*", label="goal"
    )
    ax.set(
        xlabel="X [m]", ylabel="Y [m]", zlabel="Z [m]", title="UAV 3D Motion Planning"
    )
    ax.legend(loc="upper left")
    ax.set_box_aspect((1, 1, 0.65))
    ax.view_init(elev=25, azim=-58)
    return trajectory


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("output"))
    parser.add_argument("--save", type=Path)
    args = parser.parse_args()
    fig = plt.figure(figsize=(11, 8))
    draw_scene(fig.add_subplot(111, projection="3d"), args.output)
    fig.tight_layout()
    if args.save:
        fig.savefig(args.save, dpi=160, bbox_inches="tight")
    else:
        plt.show()
