# slam_toolbox_sandbox reference

Every command in the sandbox with its arguments, plus the topics, frames and files. For what the
sandbox is, how it works, setup and step-by-step walkthroughs, start with the
[README](README.md).

**Commands:** [`sim.launch.py`](#simlaunchpy-start-the-simulation) ·
[`mapping.launch.py`](#mappinglaunchpy-build-a-slam-toolbox-map) ·
[`localization.launch.py`](#localizationlaunchpy-slam-toolbox-localization) ·
[`amcl` / `scan_match`](#amcllaunchpy-and-scan_matchlaunchpy-the-other-two-localizers) ·
[`autodrive`](#autodrive-drive-laps-automatically) ·
[`teleop`](#teleop_twist_keyboard-drive-by-hand) ·
[`png_to_world`](#png_to_world-build-a-world-from-another-map) ·
[`eval_localization.sh`](#scriptseval_localizationsh-benchmark) ·
[`plot_results.py`](#scriptsplot_resultspy-charts) · [Sanity checks](#sanity-checks) ·
[Topics and frames](#topics-and-frames) · [Files](#files)

## Command reference

All of these run in the container. Every launch argument is optional.

### `sim.launch.py`: start the simulation

```bash
ros2 launch slam_toolbox_sandbox sim.launch.py
```

Starts Gazebo headless with the track, spawns the car after 3 s, bridges its topics to ROS
(`/scan`, `/odom`, `/wheel_odom`, `/ground_truth`, `/tf`, `/clock` out; `/cmd_vel` in), publishes
the static `base_link→laser` and `world→map` transforms, and starts the Foxglove bridge.

| Argument | Default | Meaning |
|---|---|---|
| `world` | `victoria` | World file in `worlds/`, without `.sdf` |
| `x`, `y`, `yaw` | `-7.6016`, `-2.5328`, `-0.0638` | Spawn pose in the world frame (start of the Victoria centerline) |
| `gui` | `false` | Open the Gazebo window (needs working OpenGL) |
| `rviz` | `false` | Open RViz with `rviz/sandbox.rviz` (needs the VNC display) |
| `foxglove` | `true` | Start the Foxglove bridge on port 8765 |

Restart the sim between localization runs: it's the only way to put the car back at its spawn
pose.

**One copy at a time.** `sim.launch.py` refuses to start if Gazebo is already running in the same
Gazebo partition and ROS domain. The localizer launches do the same for a running localizer or
`pose_error`. Two copies would publish two cars on the same topics: nothing crashes, but TF jumps
between them and every error number is wrong. The error lists the old PIDs and the `kill -INT`
command that stops them. To run a second copy on purpose, start it with its own `IGN_PARTITION`
and `ROS_DOMAIN_ID`, as the benchmark does. The check is in `slam_toolbox_sandbox/instance_guard.py`.

### `mapping.launch.py`: build a SLAM Toolbox map

```bash
ros2 launch slam_toolbox_sandbox mapping.launch.py
```

Runs SLAM Toolbox in mapping mode (`async_slam_toolbox_node`) and `pose_error`, so you can watch
the mapping drift live.

| Argument | Default | Meaning |
|---|---|---|
| `params` | `config/mapping.yaml` | SLAM Toolbox parameters. Use `config/mapping_reference.yaml` for a sim-only near-exact map. |

Save the result with two service calls while it is still running:

```bash
# pose graph: what localization.launch.py loads -> maps/victoria_gz.posegraph + .data
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/ws/src/race_stack/utilities/slam_toolbox_sandbox/maps/victoria_gz'}"
# image: for viewing, or as map:= for AMCL / the scan matcher -> maps/victoria_gz_img.pgm + .yaml
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$HOME/ws/src/race_stack/utilities/slam_toolbox_sandbox/maps/victoria_gz_img'}}"
```

### `localization.launch.py`: SLAM Toolbox localization

```bash
ros2 launch slam_toolbox_sandbox localization.launch.py
```

Runs SLAM Toolbox in localization mode on a serialized map, plus `pose_error`.

| Argument | Default | Meaning |
|---|---|---|
| `map` | `maps/victoria_gz` | Serialized map, without extension. Use `.../maps/victoria_ref` for the committed reference map. |
| `params` | `config/localization.yaml` | SLAM Toolbox parameters |
| `x`, `y`, `yaw` | Victoria spawn pose | Must match the pose given to `sim.launch.py`: used as the start pose |
| `map_origin_x`, `_y`, `_yaw` | `0.0` | World pose of the SLAM map frame. Leave at 0 (see [Frames](README.md#frames)). |
| `csv` | empty | Path to write every error sample to |

### `amcl.launch.py` and `scan_match.launch.py`: the other two localizers

```bash
ros2 launch slam_toolbox_sandbox amcl.launch.py
ros2 launch slam_toolbox_sandbox scan_match.launch.py
```

`amcl.launch.py` starts Nav2's `map_server`, `amcl` and a lifecycle manager (they are lifecycle nodes
and need to be activated) plus `pose_error`. `scan_match.launch.py` starts `scan_match_localizer`
plus `pose_error`. The scan matcher publishes the map on `/map` once, its pose on
`/scan_match/pose` and the fraction of lidar points that land on a wall on
`/scan_match/inlier_ratio` (1.0 = perfect fit). It logs scan and rejection counts every 2 s.

| Argument | Default | Meaning |
|---|---|---|
| `map` | `stack_master/maps/Victoria/Victoria.yaml` | Map image `.yaml` (map_server format) |
| `params` | `config/amcl.yaml` / `config/scan_match.yaml` | Parameters (each file is commented) |
| `x`, `y`, `yaw` | Victoria spawn pose | Must match the pose given to `sim.launch.py` |
| `csv` | empty | Path to write every error sample to |

### `autodrive`: drive laps automatically

```bash
ros2 run slam_toolbox_sandbox autodrive --ros-args -p speed:=0.8 -p laps:=1.0
```

| Parameter | Default | Meaning |
|---|---|---|
| `speed` | `0.8` | Cruise speed [m/s]. Pass as a float (`1.0`, not `1`). |
| `laps` | `1.0` | Laps to drive, then stop. `0` = until Ctrl+C. Use ~1.1 when mapping, so the loop closes. |
| `lookahead` | `0.5` | Pure pursuit lookahead [m] |
| `waypoints` | Victoria `global_waypoints.json` | File with `centerline_waypoints` to follow |

Logs progress every 5 s and `Done: N laps` at the end.

### `teleop_twist_keyboard`: drive by hand

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`i` forward, `j` / `l` turn, `k` stop, `q` / `z` faster / slower. Keep the terminal focused while
driving.

### `png_to_world`: build a world from another map

```bash
ros2 run slam_toolbox_sandbox png_to_world \
  --map-yaml ~/ws/src/race_stack/stack_master/maps/<Map>/<Map>.yaml \
  --out ~/ws/src/race_stack/utilities/slam_toolbox_sandbox/worlds/<name>.sdf
```

| Option | Default | Meaning |
|---|---|---|
| `--map-yaml` | required | ROS occupancy map `.yaml` |
| `--out` | required | Output `.sdf` |
| `--world-name` | `track` | Gazebo world name. Keep `track`: `sim.launch.py` spawns into it. |
| `--wall-height` | `0.4` | [m] |
| `--wall-thickness` | `2` | [pixels], grown into the occupied area |
| `--render-engine` | `ogre2` | Use `ogre` if Gazebo crashes with OGRE/render errors (no GPU) |

Then rebuild the package (new file) and start the sim with `world:=<name>` and a spawn pose on
that track. The localizer and autodrive defaults still point to Victoria, so pass `map:=`, `x/y/yaw`
and `waypoints:=` too.

### `scripts/eval_localization.sh`: benchmark

```bash
cd ~/ws/src/race_stack/utilities/slam_toolbox_sandbox
./scripts/eval_localization.sh -L "scan amcl" -t compare -r 3
```

For every localizer × speed × run it starts a fresh headless sim, starts the localizer, drives
autopilot laps, stops everything and appends one row to `results/summary.csv`. It runs in its own
Gazebo partition and ROS domain (77), so an open sim session is not disturbed.

| Flag | Default | Meaning |
|---|---|---|
| `-L` | `slam` | Localizers to test: any of `slam`, `amcl`, `scan` |
| `-t` | `run` | Tag for this benchmark (used in folder names and plots) |
| `-s` | `"0.8 1.5"` | Speeds [m/s] |
| `-r` | `1` | Runs per speed, each in a fresh sim. Variance is large: use 3 or more to compare settings. |
| `-l` | `1` | Laps per run |
| `-p` | per localizer | Parameter file (needs a single `-L`) |
| `-m` | `maps/victoria_ref` for `slam`, `Victoria.yaml` for `amcl`/`scan` | Map (needs a single `-L`) |
| `-v` | off | Watch live in Foxglove. Close your own sim first: only one bridge can use port 8765. |
| `-b` | off | Record each run as an MCAP bag |
| `-i` | none | Instance 1–20 for parallel benchmarks (own partition and ROS domain 77+N). Each takes ~1 CPU core and ~1 GB. Don't combine with `-v`. |
| `-h` | | Print usage |

Output: `results/<date>_<time>[_iN]_<tag>/` with `<localizer>_speed_<v>_run<n>.{sim,loc,autodrive}.log`
and `.csv` per run, plus the row in `results/summary.csv`. The exit code is non-zero if any run
failed to start or finish its lap.

- **Start failures are retried** up to 3 attempts per run (in parallel benchmarks AMCL sometimes
  never publishes `map→odom`). The worker prints `RETRY n`, and the failed attempt's files are kept
  as `*.failed<n>`. A run that fails all 3 attempts still gets a `summary.csv` row (`lap_done`
  false, no metrics).
- **autodrive runs on sim time**, so its stuck detector (no progress for 3 s) isn't fooled by a sim
  running slower than real time. Its log ends with `wall_s=<seconds>`, for the real-time factor.

### `scripts/run_speed_sweep.sh`: speed sweep

```bash
./scripts/run_speed_sweep.sh          # start, then shows live progress
./scripts/run_speed_sweep.sh watch    # show progress again, from any container terminal
./scripts/run_speed_sweep.sh stop     # abort
```

All three localizers at 17 speeds (0.20–1.00 m/s in 0.05 steps), one 3-lap run per speed: 51 runs,
split over 3 parallel `eval_localization.sh` workers (`-i 1-3`; speeds balanced by driving time,
starts 30 s apart). About 1.5–2.5 h. Results are tagged `speedsweep`.

- **The workers are detached.** Ctrl+C or closing the terminal only stops the progress view. But the
  Mac must stay **awake, lid open**: sleep freezes Docker, and the time jump on waking spoils the runs
  in progress. Run `caffeinate -dims` on the host and keep it plugged in.
- It refuses to start if the sweep is already running, or if results with the same tag exist. Use
  `TAG=<name>` to start another one.
- Overrides for testing the tooling: `TAG`, `LAPS`, `LOCALIZERS`, `SPEEDS_1/2/3` (empty = no worker).
- Bookkeeping in `results/` (gitignored): `<tag>_worker<N>.log` (each worker's console), `<tag>.pids`,
  `<tag>_plan.json`.

The progress view (`scripts/sweep_progress.py`, tqdm) shows one overall bar and one line per worker,
with the current localizer, speed and lap. The percentage is weighted by expected driving time, so a
0.20 m/s run counts 5× a 1.0 m/s one. The ETA comes from each worker's pace so far.

### `scripts/sweep_report.py`: speed sweep table and figure

```bash
python3 scripts/sweep_report.py --tag speedsweep   # works on any tag, also mid-sweep
```

| Output | Content |
|---|---|
| `results/<tag>_table.md` / `.csv` | One row per speed + "all speeds". Per localizer: lost (lap it happened), position RMSE and p95 [cm], heading RMSE [°], lap 1→last change [cm] |
| `results/<tag>_runs.csv` | One row per run with every metric, also per lap, coverage, longest gap, real-time factor |
| `results/plots/<tag>.png` | Position RMSE, heading RMSE (log scales) and lap-to-lap change over commanded speed; whiskers = per-lap range, ✕ = lost track |

- Metrics cover only the **driving window**, from the car's first to its last movement. Every run
  logs ~10 s parked after the laps, a share that changes with speed. So these numbers are a little
  higher than the `summary.csv` ones for the same runs.
- **† flags** a run with a gap over 1 s without an estimate, an autodrive stuck event, a sim below
  0.5× real time, unfinished laps, a retried start, or an outside `/initialpose`. The reasons are
  listed under the table.
- Runs still in progress (no `summary.csv` row yet, files changing) are left out and counted.

### `scripts/plot_results.py`: charts

```bash
python3 scripts/plot_results.py --tag compare
```

Runs on the host or in the container (needs matplotlib, numpy, pyyaml, pillow). Reads
`results/summary.csv` and the per-run CSVs, writes to `results/plots/`:

| Chart | Shows |
|---|---|
| `rmse_by_run.png` | Position and heading RMSE of every 1-lap run of `--tag`, per localizer and speed, with how many lost track |
| `failures_on_track.png` | Where on the track runs lost track (error > 0.5 m) |
| `mean_by_localizer.png` | Mean / median error over lap progress per localizer (tags from `--mean-tags`, default `compare baseline`), corners shaded |
| `mean_on_track.png`, `mean_on_track_<corner>.png` | Mean estimated path per localizer on the map with its spread, and zoomed on each corner |

Runs reset by an outside `/initialpose` are flagged in `rmse_by_run.png` and left out of
`failures_on_track.png`.

### Sanity checks

```bash
ros2 topic hz /scan                                     # ~40 Hz
ros2 run tf2_ros tf2_echo odom base_link                # sim is publishing odometry
ros2 run tf2_ros tf2_echo map odom                      # a localizer is running
ros2 run rqt_plot rqt_plot /slam_eval/position_error/data   # live error plot (needs VNC)
```

---

## Topics and frames

| Topic | Type | From | Notes |
|---|---|---|---|
| `/scan` | `LaserScan` | Gazebo | 40 Hz, frame `laser` |
| `/odom` | `Odometry` | Gazebo | Exact pose, also as TF `odom→base_link` |
| `/wheel_odom` | `Odometry` | Gazebo | Diff-drive wheel odometry, unused |
| `/ground_truth` | `Odometry` | Gazebo | Exact pose in `world` |
| `/clock` | `Clock` | Gazebo | All nodes use sim time |
| `/cmd_vel` | `Twist` | autodrive / teleop | Into Gazebo |
| `/map` | `OccupancyGrid` | Localizer / map_server | |
| `/initialpose` | `PoseWithCovarianceStamped` | RViz / Foxglove pose tool | Resets the localizer |
| `/slam_eval/position_error`, `/slam_eval/yaw_error` | `Float64` | `pose_error` | [m], [rad] |
| `/scan_match/pose`, `/scan_match/inlier_ratio` | `PoseStamped`, `Float64` | Scan matcher | |

TF tree: `world → map → odom → base_link → laser`. `world→map` (identity) and `base_link→laser`
are static from `sim.launch.py`, `map→odom` comes from the localizer, and `odom→base_link` comes
from Gazebo.

## Files

| Path | What it is |
|---|---|
| `launch/sim.launch.py` | Gazebo, car, bridge, static TFs, Foxglove / RViz |
| `launch/mapping.launch.py` | SLAM Toolbox mapping + `pose_error` |
| `launch/localization.launch.py` | SLAM Toolbox localization + `pose_error` |
| `launch/amcl.launch.py` | Nav2 map_server + AMCL + `pose_error` |
| `launch/scan_match.launch.py` | `scan_match_localizer` + `pose_error` |
| `config/mapping.yaml` | SLAM Toolbox mapping parameters |
| `config/mapping_reference.yaml` | Sim-only near-exact mapping (how `maps/victoria_ref` was made) |
| `config/localization.yaml` | SLAM Toolbox localization parameters |
| `config/amcl.yaml`, `config/scan_match.yaml` | AMCL and scan matcher parameters |
| `slam_toolbox_sandbox/png_to_world.py` | Occupancy map → Gazebo world |
| `slam_toolbox_sandbox/autodrive.py` | Pure pursuit autopilot on ground truth |
| `slam_toolbox_sandbox/pose_error.py` | Error against ground truth |
| `slam_toolbox_sandbox/scan_match_localizer.py` | Fixed-map scan-to-map localizer |
| `slam_toolbox_sandbox/car_marker.py` | Draws the car at its true pose for Foxglove / RViz (`/car_marker`) |
| `slam_toolbox_sandbox/instance_guard.py` | Stops launch files from starting a second sim or localizer |
| `worlds/victoria.sdf` | Track walls generated from `stack_master/maps/Victoria` |
| `models/car.sdf` | Diff-drive car with lidar, odometry and ground truth |
| `maps/victoria_ref.*` | Committed SLAM Toolbox reference map (pose graph + image) |
| `scripts/eval_localization.sh` | Benchmark runner |
| `scripts/plot_results.py` | Benchmark charts |
| `scripts/run_speed_sweep.sh` | Speed sweep: 3 parallel benchmark workers, `watch` / `stop` |
| `scripts/sweep_progress.py` | Live tqdm progress of a sweep |
| `scripts/sweep_report.py` | Speed sweep table and figure |
| `results/summary.csv` | One row per benchmark run (committed) |
| `foxglove/sandbox_layout.json` | Foxglove layout |
| `rviz/sandbox.rviz` | RViz config |
