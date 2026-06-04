from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


def save_plots(results, cfg, results_dir):
    results_dir = Path(results_dir)
    results_dir.mkdir(exist_ok=True)

    start = np.array(cfg["start"])
    goal = np.array(cfg["goal"])

    # ============================================================
    # 1) Trajectory
    # ============================================================
    plt.figure(figsize=(8, 8))

    for mode, data in results.items():
        p = data["p"]
        plt.plot(p[:, 0], p[:, 1], linewidth=2.5, label=mode)

    plt.scatter(start[0], start[1], s=90, marker="o", label="Start")
    plt.scatter(goal[0], goal[1], s=120, marker="x", label="Goal")

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("CLF-QP Mobile Robot Trajectory")
    plt.grid(True)
    plt.axis("equal")
    plt.legend()
    plt.tight_layout()
    plt.savefig(results_dir / "trajectory_comparison.png", dpi=300)

    # ============================================================
    # 2) Distance to goal
    # ============================================================
    plt.figure(figsize=(10, 6))

    for mode, data in results.items():
        p = data["p"]
        dist = np.linalg.norm(p - goal, axis=1)
        plt.plot(data["t"], dist, linewidth=2.5, label=mode)

    plt.xlabel("Time [s]")
    plt.ylabel("Distance to goal [m]")
    plt.title("Distance to Goal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(results_dir / "distance_to_goal.png", dpi=300)

    # ============================================================
    # 3) Lyapunov function
    # ============================================================
    plt.figure(figsize=(10, 6))

    for mode, data in results.items():
        plt.plot(data["t"], data["V"], linewidth=2.5, label=mode)

    plt.xlabel("Time [s]")
    plt.ylabel("V")
    plt.title("Lyapunov Function V = 1/2 ||p - goal||^2")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(results_dir / "lyapunov_function.png", dpi=300)

    # ============================================================
    # 4) CLF constraint value
    # ============================================================
    plt.figure(figsize=(10, 6))

    for mode, data in results.items():
        plt.plot(
            data["t"],
            data["clf_constraint"],
            linewidth=2.5,
            label=mode
        )

    plt.axhline(0, linestyle="--", linewidth=1.5, label="constraint boundary")

    plt.xlabel("Time [s]")
    plt.ylabel("V_dot + cV")
    plt.title("CLF Constraint Check: V_dot + cV <= 0")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(results_dir / "clf_constraint.png", dpi=300)

    # ============================================================
    # 5) Control inputs
    # ============================================================
    plt.figure(figsize=(10, 6))

    for mode, data in results.items():
        u = data["u"]
        plt.plot(data["t"], u[:, 0], linewidth=2.0, label=f"{mode} ux")
        plt.plot(data["t"], u[:, 1], linewidth=2.0, label=f"{mode} uy")

    plt.xlabel("Time [s]")
    plt.ylabel("Control input")
    plt.title("Control Inputs")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(results_dir / "control_inputs.png", dpi=300)

    # ============================================================
    # 6) CLF correction norm
    # ============================================================
    plt.figure(figsize=(10, 6))

    for mode, data in results.items():
        plt.plot(
            data["t"],
            data["correction_norm"],
            linewidth=2.5,
            label=mode
        )

    plt.xlabel("Time [s]")
    plt.ylabel("||u_clf - u_nom||")
    plt.title("CLF-QP Correction Magnitude")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(results_dir / "clf_correction.png", dpi=300)

    print("\nSaved plots:")
    print("results/trajectory_comparison.png")
    print("results/distance_to_goal.png")
    print("results/lyapunov_function.png")
    print("results/clf_constraint.png")
    print("results/control_inputs.png")
    print("results/clf_correction.png")


def save_animation(results, cfg, results_dir):
    results_dir = Path(results_dir)
    results_dir.mkdir(exist_ok=True)

    start = np.array(cfg["start"])
    goal = np.array(cfg["goal"])

    fig, ax = plt.subplots(figsize=(8, 8))

    ax.set_title("CLF-QP Mobile Robot Control")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True)
    ax.axis("equal")

    ax.scatter(start[0], start[1], s=90, marker="o", label="Start")
    ax.scatter(goal[0], goal[1], s=120, marker="x", label="Goal")

    lines = {}
    points = {}

    for mode in results.keys():
        line, = ax.plot([], [], linewidth=2.5, label=mode)
        point, = ax.plot([], [], "o", markersize=8)
        lines[mode] = line
        points[mode] = point

    all_p = np.vstack([data["p"] for data in results.values()])
    margin = 0.8

    ax.set_xlim(all_p[:, 0].min() - margin, all_p[:, 0].max() + margin)
    ax.set_ylim(all_p[:, 1].min() - margin, all_p[:, 1].max() + margin)

    ax.legend()

    n = min(len(data["p"]) for data in results.values())
    skip = max(1, n // 250)
    frames = list(range(0, n, skip))

    def update(frame):
        for mode, data in results.items():
            p = data["p"][:frame + 1]
            lines[mode].set_data(p[:, 0], p[:, 1])
            points[mode].set_data([p[-1, 0]], [p[-1, 1]])

        return list(lines.values()) + list(points.values())

    anim = FuncAnimation(fig, update, frames=frames, interval=40, blit=True)

    gif_path = results_dir / "clf_mobile_robot_animation.gif"
    anim.save(gif_path, writer=PillowWriter(fps=25))

    print(f"Saved animation: {gif_path}")
