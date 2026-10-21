# slam_toolbox_sandbox

Standalone Gazebo (Fortress) setup for testing SLAM Toolbox mapping and localization.
It uses **nothing from the race stack**: no state estimation, controller or f1tenth_gym.
The only thing it borrows is the Victoria map, which was turned into the world file once.

```
Gazebo (world + car) ──ros_gz_bridge──► /scan, /odom, /tf (odom→base_link), /clock, /ground_truth
                                          │
                              SLAM Toolbox (map→odom)
                                          │
                      pose_error: compares map→base_link with /ground_truth
```

| File | What it is |
|---|---|
| `worlds/victoria.sdf` | Track walls generated from `stack_master/maps/Victoria` (world frame = that map's frame) |
| `models/car.sdf` | Diff-drive robot with a 270° / 1081-beam / 40 Hz lidar, wheel odometry and ground truth |
| `launch/sim.launch.py` | Gazebo + spawn + bridge + RViz |
| `launch/mapping.launch.py` | SLAM Toolbox in mapping mode |
| `launch/localization.launch.py` | SLAM Toolbox in localization mode + error measurement |
| `config/*.yaml` | SLAM Toolbox parameters |

## One-time setup (inside the container)

```bash
sudo apt update
sudo apt install -y ros-humble-ros-gz ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard
cd ~/ws
colcon build --symlink-install --packages-select slam_toolbox_sandbox
source install/setup.bash
```

## 1. Check the simulation

```bash
ros2 launch slam_toolbox_sandbox sim.launch.py
```

In a second terminal:
```bash
ros2 topic hz /scan                          # ~40 Hz
ros2 run tf2_ros tf2_echo odom base_link     # prints a transform
```

RViz opens with Fixed Frame `map`, which doesn't exist until SLAM runs. To look at the
raw sim, set Fixed Frame to `odom`: you should see the red scan outlining the walls.

If Gazebo crashes with OGRE/render errors, regenerate the world with the older engine:
```bash
ros2 run slam_toolbox_sandbox png_to_world \
  --map-yaml ~/ws/src/race_stack/stack_master/maps/Victoria/Victoria.yaml \
  --out ~/ws/src/race_stack/utilities/slam_toolbox_sandbox/worlds/victoria.sdf \
  --render-engine ogre
```

## 2. Record a map (needed once — localization needs SLAM Toolbox's own map format)

Three terminals:
```bash
# T1
ros2 launch slam_toolbox_sandbox sim.launch.py
# T2
ros2 launch slam_toolbox_sandbox mapping.launch.py
# T3 - keyboard driving: i forward, j/l turn, k stop, q/z faster/slower. Keep this terminal focused.
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Or let the autopilot drive (follows the Victoria centerline using ground truth; backs up if stuck):
```bash
ros2 run slam_toolbox_sandbox autodrive --ros-args -p speed:=0.8 -p laps:=1.1
```
`laps:=0` drives until Ctrl+C. Don't use the keyboard or Foxglove drive pad at the same time.

Drive one full lap slowly (~0.5–1 m/s) and a few metres past the start, so the loop closes.
Watch the map in RViz: when you get back to the start, the walls should snap into place with no doubled walls.

Save it (fourth terminal), then Ctrl+C everything:
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/ws/src/race_stack/utilities/slam_toolbox_sandbox/maps/victoria_gz'}"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$HOME/ws/src/race_stack/utilities/slam_toolbox_sandbox/maps/victoria_gz_img'}}"
```
You should now have `maps/victoria_gz.posegraph`, `victoria_gz.data`, and a `.pgm` + `.yaml` image.

## 3. Localize on the map

**Restart the sim** so the car is back at its spawn pose:
```bash
# T1
ros2 launch slam_toolbox_sandbox sim.launch.py
# T2
ros2 launch slam_toolbox_sandbox localization.launch.py
# T3
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

T2 prints the error against ground truth every 2 s:
```
n=812 | position: now 0.031 m, RMSE 0.045 m, max 0.120 m | yaw: now 0.40 deg, max 2.10 deg | ...
```
and a `FINAL:` summary on Ctrl+C. The first log line prints the ground-truth start pose: it should
equal the spawn pose (-7.60, -2.53). Live plot: `ros2 run rqt_plot rqt_plot /slam_eval/position_error/data`.
Add `csv:=/path/run1.csv` to the localization launch to keep every sample.

### Things to test

- **Speed:** press `q` in teleop to go faster and watch the error grow. The main knobs are
  `minimum_travel_distance` / `minimum_travel_heading` in `config/localization.yaml`: between
  corrections SLAM Toolbox only uses odometry.
- **Different start:** spawn halfway round the track, in both launches:
  ```bash
  ros2 launch slam_toolbox_sandbox sim.launch.py x:=4.264 y:=3.699 yaw:=3.513
  ros2 launch slam_toolbox_sandbox localization.launch.py x:=4.264 y:=3.699 yaw:=3.513
  ```
- **Wrong initial guess:** give only the localization launch a pose that is ~0.5 m off,
  or use RViz's **2D Pose Estimate** tool mid-run, and see whether it recovers.

`map_origin_*` in the localization launch must be the spawn pose used while *mapping*
(the default spawn, unless you changed it).

## Known limits

- The robot is diff-drive, not Ackermann. That's fine for SLAM (it only sees scans and odometry), but it turns on the spot, which a real car can't.
- Wheel odometry in Gazebo is nearly perfect, so results are optimistic. Add wheel slip or odometry noise before drawing conclusions about the real car.
- Without a GPU the lidar is rendered in software, so the sim may run slower than real time (Gazebo prints the real-time factor).
