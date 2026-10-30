"""SE(2) tracking QP adapted from the supplied agmpc_solver.py.

States are three geometric pose errors and steering angle.
Controls are speed and steering rate. Steering angle is a pure integrator;
there is no steering actuator model. No tuning defaults are copied.
"""

import time
import numpy as np


def se2_log_between(reference, current):
    """Compute Log(T_reference^-1 T_current), wrapping the heading error."""
    dx, dy = np.asarray(current[:2]) - reference[:2]
    c, s = np.cos(reference[2]), np.sin(reference[2])
    px, py = c * dx + s * dy, -s * dx + c * dy
    angle = np.arctan2(np.sin(current[2] - reference[2]),
                       np.cos(current[2] - reference[2]))
    if abs(angle) < 1e-8:
        scale = 1 - angle**2 / 12 - angle**4 / 720
    else:
        scale = 0.5 * angle / np.tan(0.5 * angle)
    return np.array([scale * px + 0.5 * angle * py,
                     -0.5 * angle * px + scale * py, angle])


class AGMPCSolver:
    def __init__(self, config):
        # Keep CasADi optional when MAP/PP are selected or AGMPC is unconfigured.
        import casadi as ca
        self.ca = ca
        self.config = config
        self.N = config['horizon']
        self.solve_time = 0.0
        self._build_optimizer()

    def _build_optimizer(self):
        ca, cfg, n = self.ca, self.config, self.N
        opti = ca.Opti('conic')
        x = opti.variable(4, n + 1)
        u = opti.variable(2, n)
        initial = opti.parameter(4)
        reference = opti.parameter(4, n + 1)
        control_reference = opti.parameter(3, n)
        dt, length = cfg['dt'], cfg['wheelbase']
        q, r = ca.DM(np.diag(cfg['state_weights'])), ca.DM(np.diag(cfg['input_weights']))
        opti.subject_to(x[:, 0] == initial)
        cost = 0
        for k in range(n):
            vd, wd = control_reference[0, k], control_reference[1, k]
            phi_ref = reference[3, k]
            # Linearized SE(2) error dynamics with phi_dot as a direct input.
            a = ca.vertcat(ca.horzcat(0, wd, 0, 0),
                           ca.horzcat(-wd, 0, vd, 0),
                           ca.horzcat(0, 0, 0, 0),
                           ca.horzcat(0, 0, 0, 0))
            omega = (ca.tan(phi_ref) / length * (u[0, k] - vd)
                     + vd / (length * ca.cos(phi_ref)**2) * (x[3, k] - phi_ref))
            inputs = ca.vertcat(u[0, k], 0, omega, u[1, k])
            opti.subject_to(x[:, k + 1] == x[:, k] + dt * (
                a @ x[:, k] + inputs + ca.vertcat(-vd, 0, 0, 0)))
            error = ca.vertcat(x[:3, k], x[3, k] - phi_ref)
            desired = ca.vertcat(vd, control_reference[2, k])
            du = u[:, k] - desired
            cost += ca.mtimes([error.T, q, error]) + ca.mtimes([du.T, r, du])
        terminal = ca.vertcat(x[:3, n], x[3, n] - reference[3, n])
        cost += cfg['terminal_weight_multiplier'] * ca.mtimes([terminal.T, q, terminal])
        opti.subject_to(opti.bounded(cfg['speed_min'], u[0, :], cfg['speed_max']))
        opti.subject_to(opti.bounded(cfg['steering_rate_min'], u[1, :], cfg['steering_rate_max']))
        opti.subject_to(opti.bounded(cfg['steering_min'], x[3, :], cfg['steering_max']))
        opti.minimize(cost)
        plugin = cfg['solver_plugin']
        opti.solver(plugin, {'printLevel': 'none'} if plugin == 'qpoases' else {})
        self.opti, self.u = opti, u
        self.initial, self.reference, self.control_reference = initial, reference, control_reference

    def solve(self, current, reference, control_reference):
        start = time.perf_counter()
        initial = np.r_[se2_log_between(reference[:, 0], current), current[3]]
        self.opti.set_value(self.initial, initial)
        self.opti.set_value(self.reference, reference)
        self.opti.set_value(self.control_reference, control_reference)
        solution = self.opti.solve()
        self.solve_time = time.perf_counter() - start
        return np.asarray(solution.value(self.u)).reshape(2, self.N)[:, 0]
