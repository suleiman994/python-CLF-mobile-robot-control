import numpy as np


def compute_metrics(data, goal, dt):
    p = data["p"]
    u = data["u"]

    goal = np.array(goal, dtype=float)

    final_distance = np.linalg.norm(p[-1] - goal)

    path_length = np.sum(
        np.linalg.norm(np.diff(p, axis=0), axis=1)
    )

    control_energy = np.sum(np.sum(u**2, axis=1)) * dt

    max_clf_violation = np.max(data["clf_constraint"])

    if len(u) > 1:
        control_smoothness = np.sum(
            np.linalg.norm(np.diff(u, axis=0), axis=1)**2
        )
    else:
        control_smoothness = 0.0

    return {
        "mode": data["mode"],
        "final_distance": final_distance,
        "path_length": path_length,
        "control_energy": control_energy,
        "control_smoothness": control_smoothness,
        "max_clf_violation": max_clf_violation,
        "final_V": data["V"][-1],
        "steps": len(data["t"]),
    }
