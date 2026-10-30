"""Use the supplied nonlinear solver with the stack's shared reference adapter."""
import numpy as np

from core.reference import PredictiveTracker, validate_config as validate_predictive_config


def validate_config(config):
    validate_predictive_config(config)
    if config['solver_plugin'] != 'ipopt':
        raise ValueError('NMPC requires the CasADi ipopt NLP plugin')


class NMPCBackend:
    """Translate the common adapter's solve interface to NonlinearMPC."""
    def __init__(self, config):
        # Imported only when NMPC is configured; other controllers do not load IPOPT.
        from .solver import NonlinearMPC
        self.optimizer = NonlinearMPC(model_config={'wheelbase': config['wheelbase']}, dt=config['dt'])
        self.optimizer.setup_solver(q=config['state_weights'], R=config['input_weights'],
                                    N=config['horizon'],
                                    terminal_state_weight_multiplier=config['terminal_weight_multiplier'])
        self.optimizer.set_control_bound(
            v_min=config['speed_min'], v_max=config['speed_max'],
            phi_min=config['steering_min'], phi_max=config['steering_max'],
            phi_dot_min=config['steering_rate_min'], phi_dot_max=config['steering_rate_max'])
        # Construct once at initialization, not inside the real-time control cycle.
        self.optimizer._build_optimizer()
        self.dt = config['dt']

    def solve(self, current, reference, controls):
        # Common controls are [v, yaw_rate, phi_dot]. NMPC directly uses [v, phi_dot].
        self.optimizer.update_ref_traj(reference, controls[[0, 2], :], self.dt)
        # Seed the nonconvex NLP near the requested forward path in map coordinates.
        # This is a reference seed, not a warm start from the previous solution.
        self.optimizer._opti.set_initial(self.optimizer._x_var, reference)
        self.optimizer._opti.set_initial(self.optimizer._u_var, controls[[0, 2], :])
        return np.asarray(self.optimizer.solve(current)).reshape(2)


class NMPCController(PredictiveTracker):
    """Same inputs, reference sampling, phi integration, and failure output as AGMPC."""
    def __init__(self, config, solver_factory=NMPCBackend):
        def factory(parameters):
            validate_config(parameters)
            return solver_factory(parameters)
        super().__init__(config, solver_factory=factory)
