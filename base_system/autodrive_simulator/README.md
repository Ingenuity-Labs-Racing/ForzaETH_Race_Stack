# AutoDRIVE Simulator

Support for running the race stack on the [AutoDRIVE RoboRacer simulator](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing).
See [`docs/autodrive_integration_checklist.md`](../../docs/autodrive_integration_checklist.md) for the integration status.

## `autodrive_roboracer`
Vendored copy of the AutoDRIVE devkit (ROS 2 bridge between the simulator and ROS).

- Upstream: `AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing`, release `2026-iros` (`autodrive_devkit.zip`, tag commit `639579a`)
- License: BSD 2-Clause, see [`autodrive_roboracer/LICENSE`](autodrive_roboracer/LICENSE) (taken from the upstream repository, the release zip ships without it)

Local changes:
- `autodrive_roboracer/autodrive_bridge.py`: the `broadcast_transforms(...)` call is commented out. Its `world -> roboracer_1 -> {imu, lidar, ...}` tree would give the `imu` frame a second parent next to the stack's `base_link -> imu`.

## `autodrive_adapter`
Makes the simulator look like the NUCNUC hardware bringup, so the rest of the stack runs unchanged.
`launch/autodrive_bringup_launch.py` is the counterpart of `f1tenth_stack/launch/bringup_launch.py` and is included by the `stack_master` launch files with `autodrive:=true`.

| Simulator | Stack | Conversion |
|---|---|---|
| `/autodrive/roboracer_1/lidar` | `/scan` | frame `laser` |
| `/autodrive/roboracer_1/imu` | `/sensors/imu/raw` | rotated into the VESC IMU mounting (`base_link -> imu` yaw +90 deg), frame `imu` |
| `/autodrive/roboracer_1/odom` | `/odom` | body velocities dead reckoned in `odom -> base_link`, like `vesc_to_odom_node` |
| `/ackermann_cmd` (ackermann_mux) | `/autodrive/roboracer_1/{throttle,steering}_command` | `speed_to_throttle_gain`, `steering_angle_to_command_gain` in `config/autodrive_adapter.yaml` |

The gains were measured on the `2026-iros` practice simulator: steady-state speed = 24.9 m/s * throttle, steering command 1.0 = 0.5236 rad.
Other measured conventions: no-return LiDAR rays are `inf`, odometry twist is in the car frame, the IMU is car aligned with gravity removed.

## Setup
Inside the race stack container:
```bash
~/ws/src/race_stack/.install_utils/autodrive_setup.sh
cd ~/ws && colcon build --packages-select autodrive_roboracer autodrive_adapter && source install/setup.bash
```

Start the simulator, either:
- the Windows build (`autodrive_simulator_practice_windows.zip` from the upstream release) on the Windows host, which reaches the container on `127.0.0.1:4567`, or
- the Linux build in Docker on the host: `~/ws/src/race_stack/.docker_utils/autodrive_sim_dock.sh`

## Usage
See the AutoDRIVE section in the [`stack_master` README](../../stack_master/README.md).
To test the bridge on its own:
1. Start the bridge: `ros2 launch autodrive_roboracer bringup_headless.launch.py`
2. In the simulator, set the vehicle to `Autonomous` and connect the bridge.
3. Drive manually: `ros2 run autodrive_roboracer teleop_keyboard`

If port 4567 is reported as in use, a previous bridge is still running: `pkill -f autodrive_bridge`.
