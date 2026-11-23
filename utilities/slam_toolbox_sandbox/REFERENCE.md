# slam_toolbox_sandbox reference

Every command in the sandbox with its arguments, plus the topics, frames and files. For what the
sandbox is, how it works, setup and step-by-step walkthroughs, start with the
[README](README.md).

**Commands:** [`sim.launch.py`](#simlaunchpy-start-the-simulation) ·
[`mapping.launch.py`](#mappinglaunchpy-build-a-slam-toolbox-map) ·
[`localization.launch.py`](#localizationlaunchpy-slam-toolbox-localization) ·
[`amcl` / `scan_match`](#amcllaunchpy-and-scan_matchlaunchpy-the-other-two-localizers) ·
[`odom_only`](#odom_onlylaunchpy-the-odometry-alone-reference) ·
[`autodrive`](#autodrive-drive-laps-automatically) ·
[`teleop`](#teleop_twist_keyboard-drive-by-hand) ·
[`png_to_world`](#png_to_world-build-a-world-from-another-map) ·
[`outline_map`](#outline_map-make-an-outline-map-for-localization) ·
[`eval_localization.sh`](#scriptseval_localizationsh-benchmark) ·
[`run_speed_sweep.sh`](#scriptsrun_speed_sweepsh-speed-sweep) ·
[`sweep_report.py`](#scriptssweep_reportpy-sweep-table-and-figures) ·
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
| `odom_noise` | `0` | Odometry noise level: `0` = perfect, `1` ≈ a real car, `2` = every error term doubled, ... (see [Odometry noise](README.md#odometry-noise)) |
| `odom_seed` | `1` | Seed of the random part of the odometry noise |

With `odom_noise` above 0, Gazebo's exact odometry is bridged to `/odom_true` (its TF to
`/odom_true/tf`, not `/tf`), and the `odom_noise` node publishes the drifting `/odom` and
`odom→base_link`. It logs its error terms at startup.

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
# pose graph: for localization.launch.py map:=... -> maps/victoria_gz.posegraph + .data
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
| `map` | `maps/victoria_ref_lc` | Serialized map, without extension, e.g. `.../maps/victoria_gz` for one you recorded |
| `params` | `config/localization_no_loop_closure.yaml` | SLAM Toolbox parameters. `config/localization.yaml` is the same with loop closure on (the default before 2026-11-22): its wrong matches move the map |
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
| `map` | `maps/victoria_outline.yaml` | Map image `.yaml` (map_server format). Use a walls-only map, see [`outline_map`](#outline_map-make-an-outline-map-for-localization). |
| `params` | `config/amcl.yaml` / `config/scan_match.yaml` | Parameters (each file is commented) |
| `x`, `y`, `yaw` | Victoria spawn pose | Must match the pose given to `sim.launch.py` |
| `csv` | empty | Path to write every error sample to |

### `odom_only.launch.py`: the odometry alone (reference)

```bash
ros2 launch slam_toolbox_sandbox odom_only.launch.py
```

No localizer: publishes a fixed identity `map→odom` and starts `pose_error`, so the error shown is
the odometry's own drift. About 0 with perfect odometry; with `sim.launch.py odom_noise:=<level>`
it shows how much drift a localizer has to remove. The benchmark's `odom` localizer. Argument:
`csv` (path to write every error sample to).

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
and `waypoints:=` too. For AMCL and the scan matcher, make an outline of the same map with
[`outline_map`](#outline_map-make-an-outline-map-for-localization) and pass that as `map:=`.

### `outline_map`: make an outline map for localization

```bash
ros2 run slam_toolbox_sandbox outline_map \
  --map-yaml ~/ws/src/race_stack/stack_master/maps/<Map>/<Map>.yaml \
  --out ~/ws/src/race_stack/utilities/slam_toolbox_sandbox/maps/<name>_outline.yaml
```

Keeps only the occupied pixels that touch free space and marks the rest of the occupied area
unknown. It uses the same rule and thickness as `png_to_world`, so the walls are exactly the pixels
the Gazebo walls are built from. Free space, resolution and origin stay the same. It writes the
`.yaml` and a `.png` next to it. `maps/victoria_outline` was made this way from `Victoria.yaml`;
[Maps](README.md#maps) explains why the localizers need it.

| Option | Default | Meaning |
|---|---|---|
| `--map-yaml` | required | Filled occupancy map `.yaml` |
| `--out` | required | Output `.yaml`; the image is written next to it as `.png` |
| `--wall-thickness` | `2` | [pixels] into the occupied area. Keep it equal to `png_to_world --wall-thickness`. |

### `scripts/eval_localization.sh`: benchmark

```bash
cd ~/ws/src/race_stack/utilities/slam_toolbox_sandbox
./scripts/eval_localization.sh -L "scan amcl" -t compare -r 3
./scripts/eval_localization.sh -L "scan odom" -t noisetest -s 1.0 -n "0 2"   # with odometry noise
```

For every localizer × speed × noise level × run it starts a fresh headless sim, starts the localizer, drives
autopilot laps, stops everything and appends one row to `results/summary.csv`. It runs in its own
Gazebo partition and ROS domain (77), so an open sim session is not disturbed.

| Flag | Default | Meaning |
|---|---|---|
| `-L` | `slam` | Localizers to test: any of `slam`, `amcl`, `scan`, and `odom` (no localizer, the odometry alone) |
| `-t` | `run` | Tag for this benchmark (used in folder names and plots) |
| `-s` | `"0.8 1.5"` | Speeds [m/s] |
| `-n` | `0` | Odometry noise levels (`sim.launch.py odom_noise`). Run r uses noise seed r, so every localizer gets the same corrupted odometry in its run r. |
| `-r` | `1` | Runs per speed and noise level, each in a fresh sim. Variance is large: use 3 or more to compare settings. |
| `-j` | none | Explicit runs `"localizer:speed:noise:run ..."` instead of `-L`/`-s`/`-n`/`-r` (how `run_speed_sweep.sh` hands out work) |
| `-l` | `1` | Laps per run |
| `-p` | per localizer | Parameter file (needs a single `-L`, not `odom`) |
| `-m` | `maps/victoria_ref_lc` for `slam`, `maps/victoria_outline.yaml` for `amcl`/`scan` | Map (needs a single `-L`, not `odom`) |
| `-v` | off | Watch live in Foxglove. Close your own sim first: only one bridge can use port 8765. |
| `-b` | off | Record each run as an MCAP bag |
| `-i` | none | Instance 1–20 for parallel benchmarks (own partition and ROS domain 77+N). Each takes ~1 CPU core and ~1 GB. Don't combine with `-v`. |
| `-h` | | Print usage |

Output: `results/<date>_<time>[_iN]_<tag>/` with `<localizer>_speed_<v>[_noise<n>]_run<r>.{sim,loc,autodrive}.log`
and `.csv` per run (`_noise<n>` only when the level isn't 0), plus the row in `results/summary.csv`
(with the actual parameter file and map, and the `odom_noise` level). The exit code is non-zero if
any run failed to start or finish its lap.

- **Start failures are retried** up to 3 attempts per run (in parallel benchmarks AMCL sometimes
  never publishes `map→odom`). The worker prints `RETRY n`, and the failed attempt's files are kept
  as `*.failed<n>`. A run that fails all 3 attempts still gets a `summary.csv` row (`lap_done`
  false, no metrics).
- **The sim check bypasses the ros2 daemon** (`ros2 topic echo --no-daemon`). A daemon whose ROS
  context has died keeps running and fails every `ros2 topic` call in its domain; before this fix
  that made worker 1 of a sweep report "sim did not start" on every attempt. `ros2 topic list`
  failing with `!rclpy.ok()` is the symptom; `ros2 daemon stop` in that domain clears it.
- **autodrive runs on sim time**, so its stuck detector (no progress for 3 s) isn't fooled by a sim
  running slower than real time. Its log ends with `wall_s=<seconds>`, for the real-time factor.

### `scripts/run_speed_sweep.sh`: speed sweep

```bash
./scripts/run_speed_sweep.sh          # start, then shows live progress
./scripts/run_speed_sweep.sh watch    # show progress again, from any container terminal
./scripts/run_speed_sweep.sh stop     # abort
```

All three localizers at 17 speeds (0.20–1.00 m/s in 0.05 steps), one 3-lap run per speed: 51 runs,
split over parallel `eval_localization.sh` workers (default 3, `-i 1..N`; runs balanced by expected
run time, longest first, starts 15 s apart). About 1.5–2.5 h. Results are tagged `speedsweep`. The
same script sweeps odometry noise levels (`NOISE`) and repeats (`REPEATS`); among runs of equal
length the localizers take turns, so the workers rarely run the heavy SLAM Toolbox at the same time.

**How many workers:** each takes ~1 CPU core and ~0.8 GB while driving with AMCL; SLAM Toolbox
needs ~2. The container has 10 cores and ~7 GB free, so 6 workers fit for one light localizer.
More barely helps: the 0.20 m/s run alone takes ~9.5 min. For one localizer, 3 workers take
~25 min, 6 ~13 min and 8 ~11 min. Six SLAM Toolbox workers ran the sim at only 0.53–0.74× real
time, which the report flags (it gives the localizer more CPU than on the car): use 5 workers
when SLAM Toolbox is in the mix.

```bash
TAG=amcl_outline LOCALIZERS=amcl WORKERS=6 ./scripts/run_speed_sweep.sh   # one localizer, 6 workers
# odometry noise test: 4 x 5 levels x 3 runs = 60 runs of 3 laps, ~25-30 min
TAG=odom_noise LOCALIZERS="scan amcl slam odom" SPEEDS=1.0 NOISE="0 1 2 4 8" REPEATS=3 WORKERS=5 ./scripts/run_speed_sweep.sh
```

- **The workers are detached.** Ctrl+C or closing the terminal only stops the progress view. But the
  Mac must stay **awake, lid open**: sleep freezes Docker, and the time jump on waking spoils the runs
  in progress. Run `caffeinate -dims` on the host and keep it plugged in.
- It refuses to start if the sweep is already running, or if results with the same tag exist. Use
  `TAG=<name>` to start another one.
- Overrides (environment variables): `TAG`, `LAPS`, `LOCALIZERS` (add `odom` for the odometry
  alone), `SPEEDS` (space-separated), `NOISE` (odometry noise levels, default `0`), `REPEATS` (runs
  per speed and noise level, default 1, seed = run number), `WORKERS` (1–20), `STAGGER` (seconds
  between worker starts), `MAP` and `PARAMS` (passed on as `-m` / `-p`; need a single localizer),
  e.g. `TAG=slam_test LOCALIZERS=slam PARAMS=config/localization.yaml WORKERS=5`.
- Bookkeeping in `results/` (gitignored): `<tag>_worker<N>.log` (each worker's console), `<tag>.pids`,
  `<tag>_plan.json`.

The progress view (`scripts/sweep_progress.py`, tqdm) shows one overall bar and one line per worker,
with the current localizer, speed, noise level, run number and lap. The percentage is weighted by expected driving time, so a
0.20 m/s run counts 5× a 1.0 m/s one. The ETA comes from each worker's pace so far.

### `scripts/sweep_report.py`: sweep table and figures

```bash
python3 scripts/sweep_report.py --tag speedsweep   # works on any tag, also mid-sweep
# combine sweeps: AMCL from a newer sweep, the other two from speedsweep -> results/amcl_outline_vs_speedsweep_*
python3 scripts/sweep_report.py --tag amcl_outline:amcl --tag speedsweep:scan,slam
```

`--tag` can be repeated; `TAG:loc,loc` takes only those localizers from that tag. Output files are
named after the tag, or the tags joined by `_vs_` (`--name` to choose). Each localizer must come
from one map: the report refuses to mix, say, AMCL on `Victoria` and on `victoria_outline`. The
table notes which map each localizer used, and SLAM Toolbox's loop-closure setting.

**Axis:** speed, or the odometry noise level when the runs have several levels (at one speed; runs
that vary both are refused). The odometry alone (`odom`) is drawn as a dashed grey reference,
not a fourth colour, and is left out of the lap-change panel, the track map and the offsets.

| Output | Content |
|---|---|
| `results/<tag>_table.md` / `.csv` | One row per speed (or noise level) + "all speeds". Per localizer: lost (lap it happened), position RMSE and p95 [cm], heading RMSE [°], lap 1→last change [cm] |
| `results/<tag>_runs.csv` | One row per run with every metric, also per lap, coverage, longest gap, real-time factor |
| `results/plots/<tag>.png` | Position RMSE, heading RMSE (log scales) and lap-to-lap change over commanded speed (or noise level); whiskers = per-lap range, ✕ = lost track |
| `results/plots/<tag>_track.png` | Track map per localizer: median position error at each point of the lap (all runs, speeds and laps), ✕ = where runs first went over 0.5 m, with counts |
| `results/plots/<tag>_heatmap.png` | One row per run (speed or noise level), one cell per ~0.37 m of the 3 laps, coloured by error: shows whether error is tied to a place on the track or to the speed, and whether a lost run recovers |
| `results/plots/<tag>_offsets.png` | Signed error in the car's frame (along, sideways, heading) per run: dot = constant part (mean), bar = ±1 SD (random part), over the samples under 0.5 m. A constant part points to setup (laser position, map alignment); one that grows with speed would be a time delay |

- Metrics cover only the **driving window**, from the car's first to its last movement. Every run
  logs ~10 s parked after the laps, a share that changes with speed. So these numbers are a little
  higher than the `summary.csv` ones for the same runs.
- **† flags** a run with a gap over 1 s without an estimate, an autodrive stuck event, a sim below
  0.9× real time (the localizer then gets more CPU per sim second than on the car), unfinished laps, a retried start, or an outside `/initialpose`. The reasons are
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
| `/odom` | `Odometry` | Gazebo, or `odom_noise` | Exact pose, also as TF `odom→base_link`; drifting with `sim.launch.py odom_noise:=<level>` |
| `/odom_true` | `Odometry` | Gazebo | Only with `odom_noise` above 0: the exact odometry that `odom_noise` corrupts |
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
from Gazebo (from `odom_noise` when the odometry is noisy).

## Files

| Path | What it is |
|---|---|
| `launch/sim.launch.py` | Gazebo, car, bridge, static TFs, Foxglove / RViz |
| `launch/mapping.launch.py` | SLAM Toolbox mapping + `pose_error` |
| `launch/localization.launch.py` | SLAM Toolbox localization + `pose_error` |
| `launch/amcl.launch.py` | Nav2 map_server + AMCL + `pose_error` |
| `launch/scan_match.launch.py` | `scan_match_localizer` + `pose_error` |
| `launch/odom_only.launch.py` | No localizer (identity `map→odom`) + `pose_error`: the odometry's own drift |
| `config/mapping.yaml` | SLAM Toolbox mapping parameters |
| `config/mapping_reference.yaml` | Sim-only near-exact mapping (how `maps/victoria_ref` was made) |
| `config/localization_no_loop_closure.yaml` | SLAM Toolbox localization parameters (default): `do_loop_closing: false`, so the map stays fixed and there are no loop-closure jumps |
| `config/localization.yaml` | The same with loop closure on (the default before 2026-11-22) |
| `config/mapping_reference_lc.yaml` | `mapping_reference.yaml` with loop closure on (how `maps/victoria_ref_lc` was made) |
| `config/amcl.yaml`, `config/scan_match.yaml` | AMCL and scan matcher parameters |
| `slam_toolbox_sandbox/png_to_world.py` | Occupancy map → Gazebo world |
| `slam_toolbox_sandbox/outline_map.py` | Filled occupancy map → walls-only map for the localizers |
| `slam_toolbox_sandbox/autodrive.py` | Pure pursuit autopilot on ground truth |
| `slam_toolbox_sandbox/pose_error.py` | Error against ground truth |
| `slam_toolbox_sandbox/odom_noise.py` | Corrupts the odometry: scale errors, gyro bias, random walk (`sim.launch.py odom_noise`) |
| `slam_toolbox_sandbox/scan_match_localizer.py` | Fixed-map scan-to-map localizer |
| `slam_toolbox_sandbox/car_marker.py` | Draws the car at its true pose for Foxglove / RViz (`/car_marker`) |
| `slam_toolbox_sandbox/instance_guard.py` | Stops launch files from starting a second sim or localizer |
| `worlds/victoria.sdf` | Track walls generated from `stack_master/maps/Victoria` |
| `models/car.sdf` | Diff-drive car with lidar, odometry and ground truth |
| `maps/victoria_ref.*` | Committed SLAM Toolbox reference map (pose graph + image), the default until 2026-11-22 |
| `maps/victoria_ref_lc.*` | The same recorded with loop closure: no doubled walls at the start line. SLAM Toolbox's default map |
| `maps/victoria_outline.*` | Walls-only Victoria map, default for AMCL and the scan matcher |
| `scripts/eval_localization.sh` | Benchmark runner |
| `scripts/plot_results.py` | Benchmark charts |
| `scripts/run_speed_sweep.sh` | Speed / odometry-noise sweep: parallel benchmark workers (`WORKERS`, default 3), `watch` / `stop` |
| `scripts/sweep_progress.py` | Live tqdm progress of a sweep |
| `scripts/sweep_report.py` | Sweep table and figures (over speed or noise level) |
| `scripts/report_figures.py` | Figures, tables and quoted numbers of the findings report, from the results (run on the host) |
| `report/main.tex`, `report/main.pdf` | Findings report, 6 pages (LaTeX). Rebuild: `python3 scripts/report_figures.py`, then `cd report && latexmk -pdf main.tex` |
| `report/main_long.tex`, `report/main_long.pdf` | The full 17-page version with every figure (same figures and tables) |
| `results/summary.csv` | One row per benchmark run (committed) |
| `foxglove/sandbox_layout.json` | Foxglove layout |
| `rviz/sandbox.rviz` | RViz config |
