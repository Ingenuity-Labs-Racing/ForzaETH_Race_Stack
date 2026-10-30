# Geometric Ackermann MPC (AGMPC)

This ROS 2 integration adapts the supplied `agmpc_solver.py` model. It uses
SE(2) pose errors, a quadratic tracking cost, and speed/steering bounds.
The optimization inputs are speed in m/s and `phi_dot` (steering rate) in rad/s.
Steering angle obeys the pure integrator `phi[k+1] = phi[k] + dt * phi_dot[k]`;
there is no steering actuator lag model. Angle and rate have separate bounds.
The adapter integrates the first optimal rate over one controller cycle and
publishes the resulting angle in radians, as required by `/drive`.

The original weights, horizon, timestep, bounds and wheelbase are **not**
reused. There is no steering time constant parameter. Vehicle and tuning values
are stored beside the AGMPC code in controller/controller/agmpc/agmpc_params.yaml.
The package installs it as share/controller/agmpc/agmpc_params.yaml. Missing required vehicle or tuning values prevent
initialization and produce zero commands. The timing thresholds max_solve_time
and input_timeout are optional: null or omitted values disable their respective
checks. If supplied, each must be finite and positive.
The config is plain YAML read by the adapter, not a ROS parameter file. Restart
the controller after editing it.

## Inputs and reference

The manager reuses `/car_state/pose`, `/car_state/odom` and `/local_waypoints`,
and monitors their receipt times together with `/state`. No new subscriptions
are introduced. It snaps to the waypoint nearest to the current position by
Euclidean distance and samples
the prediction horizon by arc length and planned speed, including measured
speed in the first prediction step. Heading is unwrapped before interpolation.
Desired steering is `atan(wheelbase * curvature)`, yaw rate is
`reference_speed * curvature`, and desired steering rate is the finite
difference of desired steering. The adapter refuses to extrapolate beyond
the local path; increase the local waypoint count if it cannot cover the horizon.

Steering angle remains a state because the bicycle kinematics need it. It
is the accumulated commanded angle, not an estimate of actuator motion.
There is no yaw-rate steering estimator. Entering AGMPC seeds the integrator
with the manager's last published angle; initial startup and stop/reset use
zero. Each accepted solve advances it by `phi_dot * (1 / controller_rate)`.
The prediction timestep and command integration timestep can differ.
The final angle is clamped to its configured bounds, and the adapter reports
the applied rate after clamping. Zero steering rate holds the commanded
angle. This assumes the vehicle follows the angle command; actual steering
feedback and actuator dynamics are not modeled.

## Optional latency compensation

`latency_compensation_s` in the AGMPC YAML sets the expected additional solve and
command-delivery delay in seconds. `0.0` disables compensation (the default).
Set it to a positive delay measured on the vehicle to enable it; desktop
benchmark times are not installed as vehicle tuning values.

When enabled, the adapter predicts the pose forward by that delay plus the
pose age, using constant measured speed and the last commanded steering angle.
It integrates constant-curvature bicycle motion without an actuator lag model.
Both nearest-waypoint selection and the initial SE(2) error use that predicted
pose. The original pose message is not modified. Reference timing and steering
command integration remain unchanged.

Pose age uses the message timestamp and the node's ROS clock. Future timestamps
cause a stop; stale timestamps cause a stop only when input_timeout is set. If a timestamp is zero, it falls back to elapsed time
since callback receipt, which does not include upstream transport/sensor delay.
The configured delay is fixed; it is not automatically adapted to solve times.
Restart the controller after editing its YAML.

## Selection

After filling a config and building/sourcing the ROS 2 workspace:

```sh
ros2 launch stack_master time_trials_launch.xml ctrl_algo:=AGMPC agmpc_config:=/absolute/path/agmpc_params.yaml
```

AGMPC is selected in the controller registry and runs under the GB_TRACK
behavior using the existing global-tracking waypoint selection. Controller
selection is independent of driving behavior.

This adapter supports global tracking only. Unsupported trailing/overtaking
behaviors produce a zero command.

Missing/invalid required configuration, missing inputs, solver exceptions,
nonfinite or out-of-bounds commands and an insufficient reference horizon
produce a zero command. input_timeout rejects old inputs before and after a
solve only when set; null allows reuse of previously received inputs without
an age limit. max_solve_time rejects late results only when set; null accepts
completed results without a solve-time limit. Neither option changes command
bounds or missing-input checks. A numeric max_solve_time does **not** interrupt
a blocking solver. A hard real-time solver watchdog
is outside this initial integration.

## Dependencies

AGMPC requires NumPy, PyYAML, CasADi and the configured CasADi conic solver plugin.
CasADi is imported only after all AGMPC configuration values pass validation;
MAP/PP and unconfigured AGMPC do not require it. Install CasADi into the Python
environment used by ROS 2, for example with `python3 -m pip install casadi`.
This integration does not install it automatically.

## Headless evaluation

Use the [two-lap evaluation mode](../../evaluation/README.md) to compare the planned
path with both driven laps and record tracking errors, commands, and solve times.
Experiment settings are separate from this folder's AGMPC tuning YAML.

For evaluation trials, select controller: AGMPC and tune controllers.AGMPC
in ../../evaluation/tuning_params.yaml. All evaluation settings are in that
single file; this folder's agmpc_params.yaml is the normal-stack preset.
