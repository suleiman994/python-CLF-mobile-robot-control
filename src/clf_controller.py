import numpy as np


def lyapunov_function(position, goal):
    """
    V = 1/2 * ||p - p_goal||^2
    """

    error = position - goal
    V = 0.5 * error @ error

    return V


def nominal_controller(position, goal, t):
    """
    Nominal controller.

    This controller moves toward the goal but it is intentionally weak
    and includes a small rotational component.

    It is not guaranteed to satisfy the CLF condition.
    """

    error = position - goal
    distance = np.linalg.norm(error)

    k_nom = 0.15

    # Weak attraction toward the goal
    u_attract = -k_nom * error

    # Rotational component around the goal
    if distance > 1e-9:
        tangent = np.array([-error[1], error[0]]) / distance
    else:
        tangent = np.zeros(2)

    u_rotate = 0.35 * np.sin(0.8 * t) * tangent

    u_nom = u_attract + u_rotate

    return u_nom


def clf_qp_controller(position, goal, t, clf_rate):
    """
    Closed-form CLF-QP controller.

    Problem:

        minimize    ||u - u_nom||^2

        subject to  V_dot <= -c V

    For single-integrator dynamics:

        p_dot = u

    with:

        V = 1/2 * e^T e
        e = p - goal

    Then:

        V_dot = e^T u

    CLF constraint:

        e^T u <= -c V

    This function projects u_nom onto the feasible half-space.
    """

    error = position - goal
    V = 0.5 * error @ error

    u_nom = nominal_controller(position, goal, t)

    # At the goal, no control is needed
    if np.linalg.norm(error) < 1e-9:
        return np.zeros(2), u_nom, V, 0.0, 0.0

    # Constraint:
    # a^T u <= b
    a = error
    b = -clf_rate * V

    # Check violation of CLF condition
    clf_value_nominal = a @ u_nom + clf_rate * V

    if clf_value_nominal <= 0.0:
        u_clf = u_nom.copy()
        correction_norm = 0.0
    else:
        # Projection onto half-space:
        # u = u_nom - ((a^T u_nom - b) / ||a||^2) * a
        violation = a @ u_nom - b
        u_clf = u_nom - (violation / (a @ a)) * a
        correction_norm = np.linalg.norm(u_clf - u_nom)

    # For checking:
    # CLF condition value should be <= 0
    clf_constraint_value = a @ u_clf + clf_rate * V

    return u_clf, u_nom, V, clf_constraint_value, correction_norm
