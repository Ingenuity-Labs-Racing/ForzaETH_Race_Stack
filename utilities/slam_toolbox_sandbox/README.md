# slam_toolbox_sandbox

A standalone Gazebo (Fortress) simulation for testing 2D lidar mapping and localization before
anything goes on the car. It was built to see whether SLAM Toolbox could replace Cartographer, and
now compares three localizers on the same track under the same conditions:

- **SLAM Toolbox** in localization mode
- **Nav2 AMCL**, a particle filter
- **`scan_match_localizer`**, a small scan-to-map matcher in this package

The sandbox uses none of the race stack's nodes (no state estimation, controller or f1tenth_gym).
It only reads two files from it: the Victoria map (`stack_master/maps/Victoria/Victoria.yaml`),
which the Gazebo world was generated from, and its centerline (`global_waypoints.json`), which
the autopilot follows.

**Contents:** [Quick start](#quick-start) · [How the simulation works](#how-the-simulation-works) ·
[Setup](#setup-once) · [Walkthroughs](#walkthroughs) · [Results so far](#results-so-far) ·
[Gaps](#gaps)

**Every command with all its arguments, plus the topic and file lists, is in
[REFERENCE.md](REFERENCE.md).**

## Quick start

Assumes the container is running and the package is built (see [Setup](#setup-once)). Open three
container terminals:

```bash
# container, terminal 1: Gazebo + the car + Foxglove bridge
ros2 launch slam_toolbox_sandbox sim.launch.py
# container, terminal 2: a localizer (scan matcher, needs no recorded map) + the error meter
ros2 launch slam_toolbox_sandbox scan_match.launch.py
# container, terminal 3: drive one lap on autopilot
ros2 run slam_toolbox_sandbox autodrive --ros-args -p speed:=0.8 -p laps:=1.0
```

On the host, open the Foxglove app, connect to `ws://localhost:8765` and import
`foxglove/sandbox_layout.json`. Terminal 2 prints the localization error every 2 s.

---

## How the simulation works

```
 ┌──────────────── Gazebo Fortress ────────────────┐
 │ worlds/victoria.sdf (walls)  models/car.sdf (car)│◄─── /cmd_vel ─── autodrive or teleop
 └────────────────────────┬─────────────────────────┘                      ▲
                          │ ros_gz_bridge                                  │ autodrive steers
       ┌──────────────────┼────────────────────┐                           │ on ground truth
       ▼                  ▼                    ▼                           │
     /scan         /odom + /tf           /ground_truth ────────────────────┘
       │          odom→base_link               │
       └────────┬─────────┘                    │
                ▼                              │
   localizer (SLAM Toolbox | AMCL | scan)      │
                │ /tf map→odom                 │
                ▼                              ▼
     pose_error: map→base_link vs. ground truth ──► /slam_eval/*, log every 2 s, CSV
```

A run always has three parts: the **sim** (`sim.launch.py`), **one localizer** (a launch file that
also starts `pose_error`) and **something driving** (`autodrive` or the keyboard).

### The world

`worlds/victoria.sdf` is generated from the Victoria occupancy map by `png_to_world`. A wall box is
placed wherever an occupied pixel touches a free one (so the black area outside the track doesn't
become one solid block), 2 pixels thick and 0.4 m high, and neighbouring wall pixels are merged
into rectangles. The Gazebo world frame is the Victoria map frame, so map coordinates (waypoints,
spawn poses) can be used directly in Gazebo.

### The car (`models/car.sdf`)

| Part | Details |
|---|---|
| Drive | Differential drive (not Ackermann): 0.3 m wheel track, 0.05 m wheels, rear ball caster. Takes `geometry_msgs/Twist` on `/cmd_vel`. |
| Lidar | Similar to a Hokuyo UST-10LX: 270°, 1081 beams, 40 Hz, 0.06–30 m, Gaussian range noise σ = 0.01 m. Frame `laser`, 0.1 m ahead of and 0.14 m above `base_link`. |
| Odometry (`/odom`, TF `odom→base_link`) | Gazebo's **exact** pose of the car, from the `OdometryPublisher` plugin. No noise, no drift. |
| Wheel odometry (`/wheel_odom`) | From the DiffDrive plugin. Published but **not used**: the wheels slip in fast turns in this simple model. |
| Ground truth (`/ground_truth`) | Also the exact pose, in the `world` frame. A separate topic so `/odom` can be made noisy later without touching the reference. |

### Frames

Odometry starts at the car's spawn pose in world coordinates, not at zero. As a result,
`world`, the origin of `odom` and `map` are all the same frame, and `sim.launch.py` publishes an
identity `world→map` transform to connect them. This is why `map_origin_*` in
`localization.launch.py` stays at 0. It would only need to be set if odometry ever started at zero
again.

Each localizer publishes `map→odom`. `pose_error` reads `map→base_link` (the localizer's estimate)
and compares it with `/ground_truth`.

### Driving

- **`autodrive`**: pure pursuit along the Victoria centerline. It steers on `/ground_truth`, so it
  drives the same line no matter what the localizer does. It slows down in sharp turns, backs up
  for 1 s if it makes no progress for 3 s, and stops after the requested number of laps.
- **`teleop_twist_keyboard`**: drive by hand from a terminal.

Only use one at a time: both publish `/cmd_vel`. Foxglove's drive pad also does.

### The three localizers

All three start from the spawn pose and publish `map→odom`.

| | SLAM Toolbox | Nav2 AMCL | `scan_match_localizer` |
|---|---|---|---|
| Launch / config | `localization.launch.py` / `config/localization.yaml` | `amcl.launch.py` / `config/amcl.yaml` | `scan_match.launch.py` / `config/scan_match.yaml` |
| Map | Its own serialized pose graph (`.posegraph` + `.data`), recorded in this sim | Map image: `Victoria.yaml` by default | Map image: `Victoria.yaml` by default |
| How it works | Adds each new scan to a short rolling buffer in its pose graph. Corrections against the saved map only happen through loop closure, which re-optimizes the graph, so **the map keeps changing** while it runs. Between corrections it uses odometry only. | Particle filter (500–2000 particles, likelihood-field model, 60 beams per update). Updates after 0.1 m or 0.1 rad of travel. Recovery is off (`recovery_alpha_* = 0`): once lost it stays lost. | For each scan (every 4th beam): predicts the pose from odometry, then refines it with Gauss-Newton so the points land on the walls of a precomputed distance field. A weak prior keeps the odometry guess along straights. A match is rejected if fewer than 30 % of points land within 0.1 m of a wall or if it moves the pose more than 0.5 m. The map is loaded once and never changes. |
| Re-initialize | RViz/Foxglove pose tool (`/initialpose`) | `/initialpose` | `/initialpose` (searches only ~0.5 m around the given pose) |

### Measuring the error (`pose_error`)

`pose_error` starts with every localization and mapping launch. For each `/ground_truth` message, it
looks up `map→base_link` at the **same timestamp**. If there's no estimate for that time the
sample is skipped and counted as skipped, because comparing against an older pose would add fake
error while the car moves.

- Publishes `/slam_eval/position_error` [m] and `/slam_eval/yaw_error` [rad].
- Logs a running summary every 2 s and a `FINAL:` line on Ctrl+C.
- With `csv:=<path>` on the launch, writes every sample (ground truth, estimate, errors) to a CSV.

The benchmark reports these numbers per run:

| Column | Meaning |
|---|---|
| `pos_rmse` | Root mean square of the position error over the run [m] |
| `pos_p95` / `pos_max` | 95th percentile / maximum position error [m] |
| `yaw_rmse_deg` / `yaw_max_deg` | Same for heading [deg] |
| `samples` | Number of compared samples (skipped ones not included) |
| `lap_done` | `true` if autodrive finished its laps before the timeout |

The plots count a run as having **lost track** once its position error exceeds 0.5 m.

### Maps

| Map | Format | What it is | Used by |
|---|---|---|---|
| `stack_master/maps/Victoria/Victoria.yaml` | Image (`.png` + `.yaml`) | The map the Gazebo world was generated from, so it matches the walls exactly | AMCL, scan matcher, `png_to_world` |
| `maps/victoria_ref` | SLAM Toolbox pose graph | A near-exact reference map, recorded with `config/mapping_reference.yaml`: the scan matcher may only move a scan ~1 cm / ~0.2° from the perfect odometry, and loop closure is off. **Simulator only**, since real odometry drifts. Committed to the repo. | SLAM Toolbox in the benchmark |
| `maps/victoria_gz` | SLAM Toolbox pose graph | The map you record yourself in [walkthrough 2](#2-record-a-slam-toolbox-map) with the normal `config/mapping.yaml`. Not in the repo. | `localization.launch.py` by default |

---

## Setup (once)

### On the host

Build and start the race stack container as described in
[`.docker_utils/README.md`](../../.docker_utils/README.md). On an Apple Silicon Mac, start it with
`.docker_utils/main_dock_mac.sh`, which forwards port 8765 for Foxglove and 5901 for VNC. Open more
container terminals with:

```bash
# host
./.docker_utils/sec_dock.sh
```

Install the [Foxglove app](https://foxglove.dev/download) on the host. It's the main way to watch
the sim.

### In the container

```bash
# container
sudo apt update
sudo apt install -y ros-humble-ros-gz ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard \
  ros-humble-foxglove-bridge ros-humble-nav2-amcl ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager ros-humble-rosbag2-storage-mcap ros-humble-rqt-plot
cd ~/ws
colcon build --symlink-install --packages-select slam_toolbox_sandbox
source install/setup.bash
```

`--symlink-install` means edits to Python nodes, configs and launch files take effect without
rebuilding. Rebuild only after adding a new file.

### Watching the sim

- **Foxglove (default).** `sim.launch.py` starts a Foxglove bridge. In the Foxglove app on the host,
  choose *Open connection* → `ws://localhost:8765`, then *Layout → Import from file* →
  `foxglove/sandbox_layout.json`.
- **RViz (fallback).** Start the sim with `rviz:=true`. RViz needs a display: on Linux, X
  forwarding from the container setup works. On a Mac, run `.docker_utils/start_vnc_display.sh`
  inside the container, then `open vnc://localhost:5901` on the host (password `racer`). Software
  rendering makes RViz slow there.

The Gazebo window is off by default (`gui:=false`): it needs working OpenGL, and nothing in the
sandbox requires it.

---

## Walkthroughs

All commands run in the container, each in its own terminal. For the arguments each one takes,
see [REFERENCE.md](REFERENCE.md).

### 1. Check the simulation

```bash
# terminal 1
ros2 launch slam_toolbox_sandbox sim.launch.py
# terminal 2
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_link
```

Expect ~40 Hz scans and a transform near `(-7.60, -2.53)`. In Foxglove, with the display frame set
to `odom`, the scan should outline the track walls. `map` only exists once a localizer runs.

If Gazebo crashes with OGRE/render errors, regenerate the world with `--render-engine ogre`
(see [`png_to_world`](REFERENCE.md#png_to_world-build-a-world-from-another-map)).

### 2. Record a SLAM Toolbox map

Only needed for SLAM Toolbox: it can't localize on a plain map image. You can skip this and use
the committed `maps/victoria_ref` instead.

```bash
# terminal 1
ros2 launch slam_toolbox_sandbox sim.launch.py
# terminal 2
ros2 launch slam_toolbox_sandbox mapping.launch.py
# terminal 3
ros2 run slam_toolbox_sandbox autodrive --ros-args -p speed:=0.8 -p laps:=1.1
```

Drive one lap slowly (0.5–1 m/s) and a few metres past the start so the loop closes. When the car
gets back to the start, the walls should snap into place with no doubled walls. Then save the map
from a fourth terminal while mapping is still running:

```bash
# pose graph: what localization.launch.py loads -> maps/victoria_gz.posegraph + .data
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/ws/src/race_stack/utilities/slam_toolbox_sandbox/maps/victoria_gz'}"
# image: for viewing, or as map:= for AMCL / the scan matcher -> maps/victoria_gz_img.pgm + .yaml
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$HOME/ws/src/race_stack/utilities/slam_toolbox_sandbox/maps/victoria_gz_img'}}"
```

Then Ctrl+C everything.

### 3. Localize with SLAM Toolbox

Restart the sim so the car is back at its spawn pose.

```bash
# terminal 1
ros2 launch slam_toolbox_sandbox sim.launch.py
# terminal 2   (add map:=$HOME/ws/src/race_stack/utilities/slam_toolbox_sandbox/maps/victoria_ref to use the reference map)
ros2 launch slam_toolbox_sandbox localization.launch.py
# terminal 3
ros2 run slam_toolbox_sandbox autodrive --ros-args -p speed:=0.8 -p laps:=1.0
```

Terminal 2 first prints the ground-truth start pose (it should equal the spawn pose), then every
2 s:

```
n=812 | position: now 0.031 m, RMSE 0.045 m, max 0.120 m | yaw: now 0.40 deg, max 2.10 deg | skipped (no SLAM pose at that time) 3
```

and a `FINAL:` line on Ctrl+C.

Things to try:

- **Speed:** drive faster (`speed:=1.5`, or `q` in teleop) and watch the error. For SLAM Toolbox the
  main knobs are `minimum_travel_distance` / `minimum_travel_heading` in `config/localization.yaml`:
  between corrections it uses odometry only.
- **Different start:** spawn halfway round the track. Pass the same pose to both launches:
  ```bash
  ros2 launch slam_toolbox_sandbox sim.launch.py x:=4.264 y:=3.699 yaw:=3.513
  ros2 launch slam_toolbox_sandbox localization.launch.py x:=4.264 y:=3.699 yaw:=3.513
  ```
- **Wrong initial guess:** give only the localization launch a pose ~0.5 m off, or use the
  pose-estimate tool mid-run, and see whether it recovers.

### 4. Compare the localizers

Same as walkthrough 3, with a different launch in terminal 2:

```bash
ros2 launch slam_toolbox_sandbox amcl.launch.py        # AMCL on Victoria.yaml
ros2 launch slam_toolbox_sandbox scan_match.launch.py  # scan matcher on Victoria.yaml
```

Both read the map image the world was generated from, so they show how good localization gets
with a perfect map. The same speed, start and initial-guess tests apply.

### 5. Run a benchmark

```bash
cd ~/ws/src/race_stack/utilities/slam_toolbox_sandbox
./scripts/eval_localization.sh -L "slam amcl scan" -t mytest -r 3 -v   # -v: watch in Foxglove
python3 scripts/plot_results.py --tag mytest
```

Each run starts a fresh headless sim in its own Gazebo partition and ROS domain, so your own sim
session isn't disturbed. Results land in `results/<date>_<time>_<tag>/` and one row per run in
`results/summary.csv`; charts go to `results/plots/`. All flags (speeds, laps, maps, parallel
instances, bag recording) are in [REFERENCE.md](REFERENCE.md#scriptseval_localizationsh-benchmark).

Don't use Foxglove's pose tool during a benchmark: it publishes `/initialpose` and resets the
localizer. To test a parameter change, copy the config, edit it and pass it with `-p` for one
localizer at a time.

---

## Results so far

From `results/summary.csv`: 1-lap runs tagged `baseline` and `compare`, 2026-10-07 to 2026-10-26,
all on Victoria with default parameters.

| Localizer | Map | Runs (0.8 / 1.5 m/s) | Median position RMSE (0.8 / 1.5) | Median heading RMSE | Runs that lost track (> 0.5 m) |
|---|---|---|---|---|---|
| Scan matcher | `Victoria.yaml` | 12 / 12 | 4.8 cm / 4.4 cm | 0.7° | 1 of 24, caused by an outside `/initialpose` reset, not by the matcher |
| AMCL | `Victoria.yaml` | 10 / 8 | 13.6 cm / 16.1 cm | 2.1–2.2° | 4 of 18, all in or entering the right-hand hairpin (x 4–6, y 1–3.5) |
| SLAM Toolbox | `victoria_ref` | 2 / 1 | 84 cm / 24 cm | 4.7–7.0° | 3 of 3 |

- The fixed-map scan matcher is the most accurate and the most consistent.
- AMCL is about 3× worse. When it loses track at the hairpin it never recovers, because recovery
  is turned off.
- SLAM Toolbox's localization mode drifts between strong map matches and is 5–20× worse. It also
  can't be made to keep its map fixed.

Charts: `results/plots/` (run `scripts/plot_results.py`). All of these numbers come with perfect
odometry: read [Gaps](#gaps) before drawing conclusions about the car.

---

## Gaps

### How the simulation differs from the car

- **Odometry is perfect.** `/odom` is Gazebo's exact pose. On the car, odometry comes from the
  VESC and IMU through an EKF and drifts, especially at speed and in slides. With perfect odometry
  and a correct start pose, a localizer that never corrects anything would score close to 0, so the
  current benchmark measures how much each localizer *disturbs* good odometry, not how well it
  *corrects* bad odometry. Adding odometry noise or bias is the most important missing piece.
- **No IMU** is simulated.
- **Diff drive, not Ackermann.** The car can turn on the spot and has no steering geometry,
  tire slip or vehicle dynamics.
- **Low speeds.** Everything so far is at 0.8 and 1.5 m/s. Race speeds are several times higher,
  where scan motion distortion and odometry errors grow.
- **Idealized lidar.** Only Gaussian range noise (σ = 0.01 m). No dropouts, reflective or
  transparent surfaces, motion distortion during a sweep, or mounting offsets. The world is flat,
  so the car never pitches or rolls.
- **The map is perfect.** The walls are generated from `Victoria.yaml`, and AMCL and the scan
  matcher localize on that same file. A real map from a mapping run is never this exact, so their
  results are an upper bound.
- **Static, empty track.** No opponent cars, people, or changes since mapping (moved boards, open
  doors).
- **One track.** Only Victoria has been tested. Other maps need a new world via `png_to_world`.
- **Localization doesn't affect driving.** `autodrive` steers on ground truth, so a bad estimate
  never makes the car drive badly. That keeps runs comparable, but it's not how the car works:
  on the car, the controller drives on the estimate.
- **Timing and CPU.** Without a GPU the lidar is rendered in software and the sim can run slower
  than real time (Gazebo prints the real-time factor). The CPU cost of each localizer hasn't been
  measured, and it matters on the NUC/Jetson.

### Tooling and open issues

- **Not connected to the race stack.** Nothing here publishes `/car_state/*` or Frenet odometry,
  so the stack's planner and controller can't run on these localizers yet.
- **No global re-localization.** The scan matcher only searches ~0.5 m around its current guess,
  and AMCL has recovery turned off. Neither recovers from a wrong start pose or from being lost.
- **Scan matcher static offset.** The scan matcher shows a steady ~3.5 cm error while the car
  stands still (start and end of a lap). This is not explained yet. A likely cause is that the
  distance field measures to wall-pixel centres, half a pixel inside the wall face.
- **AMCL startup in parallel benchmarks.** With `-i`, 6 of 18 AMCL starts never published
  `map→odom` (no errors in the logs). Serial runs didn't show this. Those runs are missing from
  the results.
- **Invalid SLAM Toolbox rows from 2026-10-28** (tag `run`, ~0.004 m RMSE). The localization log
  shows repeated `Detected jump back in time` warnings, the lap took ~11 s of wall time and more
  samples were skipped than measured. This looks like two `/clock` sources at once. Ignore these
  rows.
- **Hardcoded paths.** Launch files, `autodrive` and the benchmark assume the repo is at
  `~/ws/src/race_stack`.
- **Packages missing from the Docker images.** Both images get the Nav2 map server and lifecycle
  manager from `.install_utils/linux_req/linux_req.txt`, and `Dockerfile.arm` adds ros-gz, SLAM
  Toolbox, teleop and the Foxglove bridge. `nav2-amcl`, `rosbag2-storage-mcap` and `rqt-plot` are
  in neither image, and `Dockerfile.x86` has none of the sandbox's simulation packages. Until
  they're added, the apt line in [Setup](#in-the-container) is required.
- **Benchmark logs aren't committed.** `results/*/` is gitignored. Only `results/summary.csv` is
  shared, so the per-run CSVs and plots exist only on the machine that ran them.
