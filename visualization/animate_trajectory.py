"""Animate the UAV following the generated 3D trajectory."""
from pathlib import Path
import argparse
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from plot_trajectory import draw_scene


def drone_geometry(position, yaw, pitch, arm_length=0.55):
    """Return world-frame arms, rotor rings, and nose for a quadcopter."""
    cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
    cos_pitch, sin_pitch = np.cos(pitch), np.sin(pitch)
    rotation = np.array(
        [
            [cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch],
            [sin_yaw * cos_pitch, cos_yaw, sin_yaw * sin_pitch],
            [-sin_pitch, 0.0, cos_pitch],
        ]
    )
    center = np.asarray(position)
    diagonal = arm_length / np.sqrt(2.0)
    rotor_centers = np.array(
        [
            [diagonal, diagonal, 0.0],
            [-diagonal, -diagonal, 0.0],
            [-diagonal, diagonal, 0.0],
            [diagonal, -diagonal, 0.0],
        ]
    )

    def transform(points):
        return points @ rotation.T + center

    arms = [
        transform(rotor_centers[[0, 1]]),
        transform(rotor_centers[[2, 3]]),
    ]
    angles = np.linspace(0.0, 2.0 * np.pi, 25)
    rotor_radius = arm_length * 0.24
    ring = np.column_stack(
        (
            rotor_radius * np.cos(angles),
            rotor_radius * np.sin(angles),
            np.zeros_like(angles),
        )
    )
    rotors = [transform(ring + rotor_center) for rotor_center in rotor_centers]
    nose = transform(np.array([[0.0, 0.0, 0.0], [arm_length * 0.75, 0.0, 0.0]]))
    return arms, rotors, nose


def set_line_3d(line, points):
    line.set_data_3d(points[:, 0], points[:, 1], points[:, 2])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("output"))
    parser.add_argument("--save", type=Path)
    args = parser.parse_args()
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    tr = draw_scene(ax, args.output)
    drone_arms = [ax.plot([], [], [], color="#202020", lw=3.0)[0] for _ in range(2)]
    drone_rotors = [ax.plot([], [], [], color="#00bcd4", lw=1.8)[0] for _ in range(4)]
    (drone_nose,) = ax.plot([], [], [], color="#ffcc00", lw=3.0)
    (trail,) = ax.plot([], [], [], color="#17becf", lw=2)

    def update(i):
        position = [tr["x"][i], tr["y"][i], tr["z"][i]]
        arms, rotors, nose = drone_geometry(position, tr["yaw"][i], tr["pitch"][i])
        for line, points in zip(drone_arms, arms):
            set_line_3d(line, points)
        for line, points in zip(drone_rotors, rotors):
            set_line_3d(line, points)
        set_line_3d(drone_nose, nose)
        trail.set_data_3d(tr["x"][: i + 1], tr["y"][: i + 1], tr["z"][: i + 1])
        return (*drone_arms, *drone_rotors, drone_nose, trail)

    animation = FuncAnimation(
        fig,
        update,
        frames=range(0, len(tr["time"]), 2),
        interval=50,
        blit=False,
        repeat=True,
    )
    if args.save:
        animation.save(args.save, fps=20)
    else:
        plt.show()
