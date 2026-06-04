import numpy as np

from src.clf_controller import (
    nominal_controller,
    clf_qp_controller,
    lyapunov_function,
)


def simulate(mode, cfg):
    """
    mode:
        nominal
        clf_qp
    """

    dt = cfg["dt"]
    sim_time = cfg["sim_time"]
    n_steps = int(sim_time / dt)

    start = np.array(cfg["start"], dtype=float)
    goal = np.array(cfg["goal"], dtype=float)

    position = start.copy()

    t_log = []
    p_log = []
    u_log = []
    v_log = []
    clf_constraint_log = []
    correction_log = []

    for k in range(n_steps):
        t = k * dt

        if mode == "nominal":
            u = nominal_controller(position, goal, t)
            V = lyapunov_function(position, goal)

            error = position - goal
            clf_constraint_value = error @ u + cfg["clf_rate"] * V
            correction_norm = 0.0

        elif mode == "clf_qp":
            u, u_nom, V, clf_constraint_value, correction_norm = clf_qp_controller(
                position=position,
                goal=goal,
                t=t,
                clf_rate=cfg["clf_rate"]
            )

        else:
            raise ValueError("Unknown mode")

        # Single-integrator dynamics:
        # p_dot = u
        position = position + dt * u

        t_log.append(t)
        p_log.append(position.copy())
        u_log.append(u.copy())
        v_log.append(V)
        clf_constraint_log.append(clf_constraint_value)
        correction_log.append(correction_norm)

        if np.linalg.norm(position - goal) < cfg["goal_tolerance"]:
            # Continue logging a little is not necessary
            break

    data = {
        "mode": mode,
        "t": np.array(t_log),
        "p": np.array(p_log),
        "u": np.array(u_log),
        "V": np.array(v_log),
        "clf_constraint": np.array(clf_constraint_log),
        "correction_norm": np.array(correction_log),
    }

    return data
