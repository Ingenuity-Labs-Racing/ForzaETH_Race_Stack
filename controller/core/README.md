# Controller framework

The controller ROS package contains:
- core/: ROS manager, interface, registry, shared reference/tracking mechanics.
- controller/map/, pp/, ftg/, agmpc/, nmpc/: algorithms and their adapters.
- controller/pursuit.py: shared MAP/PP adapter and L1 parameter handling.
- evaluation/: two-lap runner, configuration, metrics and plots.
- evaluation/visualization/: preserved experiment recordings and figures.

The state machine publishes driving behaviors GB_TRACK, TRAILING, OVERTAKE,
and FTGONLY. It selects waypoints and behavior transitions; it never chooses
the controller algorithm. Legacy NMPC_TRACK inputs map to GB_TRACK. Existing state indices 0 through 3 are unchanged.

AGMPC is the geometric-controller selection name. Its code and tuning YAML
are under controller/agmpc/. NMPC uses controller/nmpc/.

## Add a controller

1. Add a Python package under controller/<name>/ containing an adapter
   implementing ControllerAdapter.reset() and step(ControllerInputs), a config
   loader, and the algorithm and YAML.
2. Add one ControllerSpec entry in core/controller_registry.py identifying
   the factory, loader, default installed config path, required inputs,
   supported behaviors, and optional report capabilities.
3. Select its name and tune its controllers.<NAME> block in evaluation/tuning_params.yaml.

Package discovery installs new controller packages and their YAML files
automatically. The manager, state-machine code, and evaluator need no new
algorithm branches. Rebuild the controller ROS package after adding modules;
YAML-only changes need a new launch.

ControllerInputs contains pose [x,y,yaw], speed, local waypoints, normalized
behavior, command timestep and pose age, plus optional Frenet state,
acceleration, opponent/obstacles, track length and LiDAR. Existing waypoint
columns remain x, y, speed, track ratio, s, curvature, heading, acceleration.
ControllerResult returns speed in m/s, steering angle in radians, validity,
reason, optional steering rate and diagnostics.

Required input cache names are pose, velocity, waypoints, state, frenet,
track, scan, obstacles and acceleration. Unsupported behaviors, missing/stale
required inputs, exceptions and invalid commands produce zero commands.
MAP/PP explicitly route FTGONLY to the registered FTG adapter; predictive
controllers have no automatic fallback into unsupported behaviors.

Optional report capabilities solve_time and constraints enable the relevant
metrics and plot limits. Adapters supply reference diagnostics and the registry
supplies plot labels; there is no evaluator switch per algorithm. FTG has no
waypoint reference or solve metrics, so those values are unavailable.
AGMPC and NMPC use independent trackers with shared reference interpolation,
latency compensation, steering-command integration and failure checks.

## ROS usage

    ros2 launch stack_master time_trials_launch.xml racecar_version:=SIM ctrl_algo:=AGMPC

The generic controller_config launch argument can select any registry
controller's YAML. Controller-specific agmpc_config, nmpc_config and map_config arguments
are also available. The state-machine initial_state defaults to GB_TRACK for all
controllers.

    ros2 launch stack_master controller_tuning.launch.py

This reads controller/evaluation/tuning_params.yaml in the source checkout.
Reports default to controller/evaluation/visualization/<unique run>/.
See ../evaluation/README.md for the recording workflow.

Evaluation uses one self-contained YAML. ControllerSpec.parameters() extracts
controllers.<NAME> when present, and normal stack launches still accept the
adapter's standalone preset format. Config loaders should accept flat parameter
mappings for inline blocks. Each run records the complete experiment YAML and
a resolved selected-controller snapshot.
