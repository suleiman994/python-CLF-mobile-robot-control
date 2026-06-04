from pathlib import Path

import numpy as np

from src.simulate import simulate
from src.metrics import compute_metrics
from src.plot_results import save_plots, save_animation


PROJECT_DIR = Path(__file__).resolve().parent
RESULTS_DIR = PROJECT_DIR / "results"
RESULTS_DIR.mkdir(exist_ok=True)


def main():
    cfg = {
        "dt": 0.02,
        "sim_time": 15.0,

        "start": [0.0, 0.0],
        "goal": [5.0, 5.0],

        # CLF rate c in V_dot <= -cV
        "clf_rate": 0.8,

        "goal_tolerance": 0.03,
    }

    print("\n========== CLF-QP Mobile Robot Control ==========")
    print("State: p = [x, y]")
    print("Dynamics: p_dot = u")
    print("Goal:", cfg["goal"])
    print("\nLyapunov function:")
    print("V = 1/2 * ||p - goal||^2")
    print("\nCLF condition:")
    print("V_dot <= -cV")
    print("Equivalent constraint:")
    print("(p - goal)^T u <= -c * V")

    modes = ["nominal", "clf_qp"]

    results = {}

    for mode in modes:
        print(f"\nRunning mode: {mode}")
        results[mode] = simulate(mode, cfg)

    print("\n========== Metrics ==========")

    for mode in modes:
        m = compute_metrics(
            data=results[mode],
            goal=cfg["goal"],
            dt=cfg["dt"]
        )

        print(f"\nMode: {m['mode']}")
        print(f"Final distance to goal: {m['final_distance']:.4f} m")
        print(f"Path length: {m['path_length']:.4f} m")
        print(f"Control energy: {m['control_energy']:.4f}")
        print(f"Control smoothness: {m['control_smoothness']:.4f}")
        print(f"Max CLF violation V_dot + cV: {m['max_clf_violation']:.8f}")
        print(f"Final Lyapunov V: {m['final_V']:.8f}")
        print(f"Steps: {m['steps']}")

    save_plots(results, cfg, RESULTS_DIR)
    save_animation(results, cfg, RESULTS_DIR)

    print("\n========== Finished ==========")
    print("Results saved in results/ folder.")


if __name__ == "__main__":
    main()
