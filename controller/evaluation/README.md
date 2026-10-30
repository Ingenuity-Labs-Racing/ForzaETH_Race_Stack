# Headless two-lap controller evaluation

This mode runs one simulation trial of two continuous laps. It resets the
vehicle and selected controller once before the trial. There is no reset, pause, or controller
restart between laps. The first lap includes acceleration from standstill.

## Configuration and execution

Edit **tuning_params.yaml beside this file** to select the map and controller,
tune the controller, and set the starting waypoint, output location and timeouts.
This is the only source configuration file needed for an evaluation.

```yaml
controller: AGMPC

controllers:
  MAP:
    lookup_table: NUC2_hangar_pacejka
    # Full MAP parameters are already populated in the actual file.
  AGMPC:
    # Full AGMPC parameters are already populated.
  NMPC:
    # Full NMPC parameters are already populated.
  PP:
    # Full PP parameters are already populated.
  FTG:
    # Full FTG parameters are already populated.
```

The snippet shows the structure, not a replacement for the complete file.
Change `controller` to select a block, then edit that block's parameters.
There is no separate `controller_config` path. Only the selected block is
validated and loaded; the other blocks do not affect the run.

The initial values were copied from the existing controller presets without
retuning. Later changes in this file affect evaluation trials only. Normal
stack launches still use their own preset YAMLs. Explicit inline FTG settings
also take precedence over the state machine's default FTG parameters.

All controllers run under GB_TRACK. The registry still supplies the adapter,
required inputs, behavior support, and report capabilities. New controllers
need their own parameter block here and a registry entry; the evaluator does
not need a new algorithm branch.

MAP's lookup_table is inside controllers.MAP. It selects an installed
steering_lookup/cfg/<name>_lookup_table.csv. AGMPC and NMPC weights, horizons,
limits, and optional timing settings are independent blocks.

Inside the existing ROS 2 container, rebuild after adding this mode:

    cd ~/ws
    colcon build --symlink-install --packages-select controller state_machine stack_master
    source install/setup.bash
    ros2 launch stack_master controller_tuning.launch.py

No DISPLAY, RViz, noVNC, GPU or Docker image rebuild is needed. Keep other
simulator/controller launches stopped. The evaluator checks for conflicting
drive publishers and verifies the global simulation flag before enabling
commands or resetting the vehicle. The launch always selects SIM.

The default configuration is read from the mounted source checkout. Paths in
the YAML are relative to source_root, discovered from the source launch path
or ~/ws/src/race_stack. An alternative checkout can be selected with:

    ros2 launch stack_master controller_tuning.launch.py source_root:=/absolute/path/race_stack

evaluation_config and output_directory launch arguments can override the
experiment YAML path and output location. The map glc_ot_ez has a
grayscale image compatible with the existing simulator. RGB/RGBA maps are
rejected before startup; this feature does not modify the simulator submodule.

## Results

Each trial creates a unique UTC timestamp/ID directory under
controller/evaluation/visualization in the source checkout, visible from Windows.

- experiment.yaml: exact bytes of the complete unified experiment YAML.
- controller_params.yaml: resolved parameters from the selected controller block.
- planned_path.csv: scaled global reference waypoints used for the run.
- samples.csv: streamed, flushed per-command data, with lap and cycle IDs.
- summary.json: overall and per-lap metrics, durations, failures, sample gaps,
  constraint usage, solve times, completion reason and metric definitions.
- planned_path.png: complete path colored by desired waypoint speed.
- trajectory_comparison.png: three vertically stacked, shared-scale panels:
  both laps, lap one, lap two, each overlaid on the planned path.
- tracking_and_commands.png: four aligned time plots: Euclidean position
  error, wrapped heading error in degrees, reference/command/measured speed, and
  reference/command steering. The lap boundary is marked; configured steering limits
  are shown for AGMPC and NMPC runs.

References are captured in the same controller cycle as each published
command, not reconstructed afterward. The physical pose is retained
separately from the latency-compensated prediction pose. Position error is
distance to the recorded reference waypoint, not segment distance.
For AGMPC and NMPC this is the first time-sampled controller reference. For MAP it is
the local planner waypoint nearest to the measured pose in Euclidean distance,
recorded before MAP runs. Its desired speed is recorded before any MAP speed
reduction or lookahead. Planner-derived steering is atan(L * curvature), using
the simulator wheelbase L = lf + lr, and is not MAP's lookup-table output.
This makes the plot a comparison of the planner's desired motion with MAP's
published commands. Measured speed is plotted separately. MAP's nearest
reference is not its L1 pursuit point or speed-lookahead target.
MAP reports set solve_time_s, solver_failures and saturation percentages to
null because MAP has no AGMPC solve or configured AGMPC input bounds. Simulator
actuator limits are not presented as MAP controller limits.
Steering is a command, not measured actuator feedback. Statistics are
sample-based and report finite sample counts and observed recording intervals.

Lap counting uses signed unwrapped Frenet progress relative to the reset
position. An initial finish-line crossing does not count as a full lap;
backward motion does not add progress. Large position jumps abort the trial.
The old lap analyser is left unchanged because it discards an initial partial
lap, which is not the desired start-from-rest experiment definition.

Completion pauses the controller and publishes zero speed/steering before
report generation. Controller errors, data loss, startup/reset/trial timeouts
and lack of forward progress save an incomplete report rather than waiting
forever for lap completion. Simulator collisions are not directly exposed by
the existing bridge: collision_count is null, never an invented count.
A stalled vehicle is reported as stalled, not asserted to be a collision.
An unresponsive stop service is reported with stop_confirmed: false; launch
shutdown also terminates the simulated vehicle process.

Reports can be regenerated inside the container without rerunning simulation:

    ros2 run controller controller_evaluation_plot /absolute/path/to/experiment

Changing the single tuning YAML needs a fresh evaluation launch, not a Docker
rebuild. Packaging/entry-point changes need the selected ROS packages rebuilt.

The existing simulator bridge and some stack nodes print shutdown tracebacks
when the evaluator ends the launch. Check summary.json for trial completion,
stop confirmation, and report generation status; those shutdown messages are
separate from AGMPC solve failures. The simulator submodule is unchanged.

Heading error is saved as heading_error_deg in samples.csv and summary.json.
Pose/reference headings and steering angles retain their existing radians units.
The plot tool also converts heading errors from older recordings to degrees.
