# Nonlinear MPC

This adapter integrates the supplied solver.py and
symbolic_system.py model. It retains the nonlinear Cartesian bicycle dynamics:

    x_dot = v * cos(yaw)
    y_dot = v * sin(yaw)
    yaw_dot = v * tan(phi) / wheelbase
    phi_dot = steering_rate

The supplied cost form is unchanged: Cartesian position error, periodic
heading/steering costs, and speed/steering-rate reference error, plus terminal
state cost. The NMPC solves with CasADi/IPOPT, not the AGMPC conic QP plugin.

## Configuration

nmpc_params.yaml is independent of agmpc/agmpc_params.yaml. Vehicle settings use
SIM wheelbase 0.3302 m, command period 0.025 s, steering +/-0.4 rad, steering
rate +/-3.2 rad/s, horizon 20, and forward speed 0 to 10 m/s. The speed bound is
a ceiling, not a request to drive at that speed.

Initial state weights [200, 200, 0, 0], input weights [0.8, 0.8], and terminal
multiplier 100 come from the supplied NMPC defaults. They are starting values,
not a tuned configuration for this simulator. Unlike AGMPC, its state weights
refer to Cartesian x/y, heading, and steering; identical weights would not
mean identical costs between controllers.

The adapter supplies the wheelbase explicitly, replacing the standalone
model's original 0.256 m assumption. It supplies separate speed/steering-rate
weights and enforces steering-rate bounds inside the optimizer as well as the
existing command checks. The provided solver's IPOPT options remain unchanged.

## Stack integration

NMPC takes the existing map pose, odometry speed, and local waypoint array.
It uses shared predictive reference sampling: snap to the nearest waypoint by
Euclidean distance, then advance along path distance using desired speeds and
dt for N+1 state references and N controls. The common reference has
[v, yaw_rate, phi_dot]; the nonlinear solver receives [v, phi_dot].

Steering feedback is the accumulated command, not measured steering.
The output is speed in m/s and integrated steering angle in radians, with
validity diagnostics. There is no steering-actuator lag model. Optional
latency compensation and timing thresholds behave as for AGMPC.

The NLP is built once when NMPC initializes. Each solve updates the initial
state and reference parameters. Initial guesses are seeded from the current
reference trajectory, rather than zeros in map coordinates. No previous-solution
warm-start sequence is added.
Missing inputs, invalid references/configuration, solver errors, and invalid
commands produce zero commands through the shared adapter.

NMPC is selected through the controller registry and uses GB_TRACK like
AGMPC and MAP. There is no NMPC-specific driving state. The adapter supports
global tracking only; unsupported behaviors produce a zero command.
Its tracker shares solver-independent reference mechanics with AGMPC and
does not inherit from the geometric-controller implementation.

## Run

Inside the ROS 2 container, build/supply the updated packages once:

    cd ~/ws
    colcon build --symlink-install --packages-select controller state_machine stack_master
    source install/setup.bash

For an already running simulator:

    ros2 launch stack_master time_trials_launch.xml racecar_version:=SIM ctrl_algo:=NMPC

For the automatic two-lap mode, select controller: NMPC in
../../evaluation/tuning_params.yaml and tune controllers.NMPC in that same file.
The standalone nmpc_params.yaml remains the normal-stack preset.

Then:

    ros2 launch stack_master controller_tuning.launch.py

The two-lap mode launches its own headless simulator and saves all plots,
CSV samples, and per-lap metrics. NMPC reports include solve timing and
configured constraint usage. Heading error is in degrees; steering is radians.
See ../../evaluation/README.md for the report format and stop behavior.
YAML changes require a new launch, not a Docker image rebuild.
