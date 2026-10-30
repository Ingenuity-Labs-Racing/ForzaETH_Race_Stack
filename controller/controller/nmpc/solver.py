import numpy as np
import casadi as ca
from .enum_class import (
    CostType,
    DynamicsType,
    ControllerType,
)
from .symbolic_system import FirstOrderModel
import time

"""
naive MPC for unicycle model
"""


class BycicleModel:
    def __init__(self, config: dict = {}, dt = 0.05, **kwargs):
        self.nState = 4
        self.nControl = 2

        self.dt = dt
        self.wheelbase = float(config.get('wheelbase', 0.256))

        # setup configuration
        self.config = {'cost_type': CostType.POSITION_EULER, 'dynamics_type': DynamicsType.EULER_FIRST_ORDER}
        if self.config['dynamics_type'] == DynamicsType.EULER_FIRST_ORDER:
            self.set_up_euler_first_order_dynamics()

        print(config.get("dynamics_type"))

    def set_up_euler_first_order_dynamics(self):
        print("Setting up Euler first order dynamics")
        nx = self.nState
        nu = self.nControl
        # state
        x = ca.MX.sym('x')
        y = ca.MX.sym('y')
        theta = ca.MX.sym('theta')
        phi = ca.MX.sym('phi')
        X = ca.vertcat(x, y, theta, phi)
        # controlcost

        v = ca.MX.sym('v')  # linear velocity
        steering_ratio = ca.MX.sym('steering_ratio')  # angular velocity
        U = ca.vertcat(v, steering_ratio)

        # state derivative
        x_dot = ca.cos(theta) * v
        y_dot = ca.sin(theta) * v
        theta_dot = v * ca.tan(phi) / self.wheelbase
        phi_dot = steering_ratio
        X_dot = ca.vertcat(x_dot, y_dot, theta_dot, phi_dot)

        # cost function
        self.costType = self.config['cost_type']
        print("Cost type: {}".format(self.costType))
        Q = ca.MX.sym('Q', nx, nx)
        R = ca.MX.sym('R', nu, nu)
        Xr = ca.MX.sym('Xr', nx, 1)
        Ur = ca.MX.sym('Ur', nu, 1)
        if self.costType == CostType.POSITION:
            cost_func = 0.5 * (X[:2] - Xr[:2]).T @ Q[:2, :2] @ (X[:2] - Xr[:2]) + 0.5 * (U - Ur).T @ R @ (U - Ur)
        elif self.costType == CostType.POSITION_EULER:
            pos_cost = 0.5 * (X[:2] - Xr[:2]).T @ Q[:2, :2] @ (X[:2] - Xr[:2])
            theta = X[2]
            theta_target = Xr[2]
            dth = theta - theta_target
            qth = Q[2,2]

            euler_cost = 0.5*qth*(1 - ca.cos(dth))
        
            phi = X[3]
            phi_target = Xr[3]
            dphi = phi - phi_target
            qphi = Q[3,3]

            euler_phi_cost = 0.5*qphi*(1 - ca.cos(dphi))
            cost_func = pos_cost + euler_cost + euler_phi_cost + 0.5 * (U - Ur).T @ R @ (U - Ur)

        cost = {'cost_func': cost_func, 'vars': {'X': X, 'Xr': Xr, 'U': U, 'Ur': Ur, 'Q': Q, 'R': R}}
    
        # define dynamics and cost dict
        dynamics = {'dyn_eqn': X_dot, 'vars': {'X': X, 'U': U}}
        params = {
            'X_EQ': np.zeros(self.nState),  # np.atleast_2d(self.X_GOAL)[0, :],
            'U_EQ': np.zeros(self.nControl)  # np.atleast_2d(self.U_GOAL)[0, :],
        }
        self.symbolic = FirstOrderModel(dynamics, cost, self.dt, params)
    
class NonlinearMPC:
    def __init__(self, model_config={}, dt=0.05):
        config = model_config
        # dynamics
        self.model = BycicleModel(config, dt=dt).symbolic
        self.ref_state = None
        self.ref_control = None
        self.nState = self.model.nx  # 4 (x, y, theta,phi)
        self.nControl = self.model.nu  # 2 (v, steering_ratio)
        self.solve_time = 0.0
        self._opti = None
        self.setup_solver()
        self.set_control_bound()
        self.cost_func = self.model.cost_func

    def setup_solver(
        self,
        q=[200, 200, 0, 0],
        R=0.8,
        N=10,
        terminal_state_weight_multiplier=100.0,
    ):
        self.Q = np.diag(q)
        weights = np.asarray(R, dtype=float)
        self.R = float(weights) * np.eye(self.model.nu) if weights.ndim == 0 else np.diag(weights)
        self.N = N
        self.terminal_state_weight_multiplier = float(
            terminal_state_weight_multiplier
        )
        # Build lazily after the adapter has supplied the final weights and
        # horizon. Calling setup_solver again invalidates the existing NLP.
        self._opti = None

    def set_control_bound(self, v_min = -1.75, v_max= 1.75, phi_min = -0.5, phi_max = 0.5,
                          phi_dot_min=None, phi_dot_max=None):
        self.v_min = v_min
        self.v_max = v_max
        self.phi_min = phi_min
        self.phi_max = phi_max
        self.phi_dot_min = phi_dot_min
        self.phi_dot_max = phi_dot_max
        self._opti = None

    def _build_optimizer(self):
        """Build the parameterized nonlinear program once."""
        nu = self.nControl
        nx = self.nState
        N = self.N

        opti = ca.Opti()
        x_var = opti.variable(nx, N + 1)
        u_var = opti.variable(nu, N)

        initial_state = opti.parameter(nx)
        ref_state = opti.parameter(nx, N + 1)
        ref_control = opti.parameter(nu, N)
        dt = opti.parameter()
        v_min = opti.parameter()
        v_max = opti.parameter()
        phi_min = opti.parameter()
        phi_max = opti.parameter()

        opti.subject_to(x_var[:, 0] == initial_state)

        for i in range(N):
            x_next = x_var[:, i] + dt * self.model.fc_func(
                x_var[:, i],
                u_var[:, i],
            )
            opti.subject_to(x_var[:, i + 1] == x_next)

        Q = ca.DM(self.Q)
        R = ca.DM(self.R)
        cost = 0
        for i in range(N):
            cost += self.cost_func(
                x_var[:, i],
                ref_state[:, i],
                u_var[:, i],
                ref_control[:, i],
                Q,
                R,
            )

        zero_control = ca.DM.zeros(nu, 1)
        cost += self.cost_func(
            x_var[:, N],
            ref_state[:, N],
            zero_control,
            zero_control,
            self.terminal_state_weight_multiplier * Q,
            R,
        )

        opti.subject_to(u_var[0, :] >= v_min)
        opti.subject_to(u_var[0, :] <= v_max)
        opti.subject_to(x_var[3, :] >= phi_min)
        opti.subject_to(x_var[3, :] <= phi_max)

        if self.phi_dot_min is not None and self.phi_dot_max is not None:
            opti.subject_to(opti.bounded(self.phi_dot_min, u_var[1, :], self.phi_dot_max))

        opti.minimize(cost)
        opts_setting = {
            'ipopt.max_iter': 1000,
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.acceptable_tol': 1e-8,
            'ipopt.acceptable_obj_change_tol': 1e-6,
        }
        opti.solver('ipopt', opts_setting)

        self._opti = opti
        self._x_var = x_var
        self._u_var = u_var
        self._initial_state_param = initial_state
        self._ref_state_param = ref_state
        self._ref_control_param = ref_control
        self._dt_param = dt
        self._v_min_param = v_min
        self._v_max_param = v_max
        self._phi_min_param = phi_min
        self._phi_max_param = phi_max

    def update_ref_traj(self, ref_state, ref_control, dt):
        self.ref_state = ref_state
        self.ref_control = ref_control
        self.dt = dt
        # Keep the model metadata synchronized with the runtime command
        # period. The optimizer uses ``self.dt`` as its discretization
        # parameter, while other consumers may inspect ``model.dt``.
        self.model.dt = dt

    def solve(self, state):
        """
        state: [x, y, theta,phi]
        t: time -> index of reference trajectory (t = k * dt)
        """
        start_time = time.time()
        if self.ref_state is None:
            raise ValueError('Reference trajectory is not set up yet!')

        if self._opti is None:
            self._build_optimizer()

        self._opti.set_value(self._initial_state_param, state)
        self._opti.set_value(self._ref_state_param, self.ref_state)
        self._opti.set_value(self._ref_control_param, self.ref_control)
        self._opti.set_value(self._dt_param, self.dt)
        self._opti.set_value(self._v_min_param, self.v_min)
        self._opti.set_value(self._v_max_param, self.v_max)
        self._opti.set_value(self._phi_min_param, self.phi_min)
        self._opti.set_value(self._phi_max_param, self.phi_max)

        sol = self._opti.solve()
        u = sol.value(self._u_var[:, 0])
        self.solve_time = time.time() - start_time
        return u

    def get_solve_time(self):
        return self.solve_time

    def vel_cmd_to_local_twist(self, vel_cmd):
        return ca.vertcat(vel_cmd[0], 0, vel_cmd[1])

    def local_twist_to_vel_cmd(self, local_vel):
        return ca.vertcat(local_vel[0], local_vel[2])

