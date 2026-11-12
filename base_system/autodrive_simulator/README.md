# AutoDRIVE Simulator

Runs the race stack on the [AutoDRIVE RoboRacer simulator](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing), on any map in `stack_master/maps/`. The simulator only replaces the car hardware: the stack runs with the real car pipeline (NUCNUC config, Cartographer and EKF).

| Section | Description |
|---|---|
| [Run with a Map](#run-with-a-map) | Setup, then the steps for every run. |
| [Troubleshooting](#troubleshooting) | Common problems. |
| [Build the Simulator](#build-the-simulator) | Add the map loader to the AutoDRIVE Unity project (only if the team build is not available). |
| [Reference](#reference) | Packages, topics and local changes. |


## Run with a Map

### One-Time Setup
1. Inside the race stack container, install the bridge dependencies and build:
   ```bash
   ~/ws/src/race_stack/.install_utils/autodrive_setup.sh
   cd ~/ws && colcon build --packages-select autodrive_roboracer autodrive_adapter stack_master && source install/setup.bash
   ```
2. Get the Windows simulator build with the map loader from the team. The released AutoDRIVE simulator (and `.docker_utils/autodrive_sim_dock.sh`) ignores the map and always drives its built-in track. To build it yourself, see [Build the Simulator](#build-the-simulator).

### Every Run
1. In the container, launch the stack with a map (here `Victoria`). This also starts the AutoDRIVE bridge on port 4567:
   ```bash
   ros2 launch stack_master base_system_launch.xml autodrive:=true map_name:=Victoria
   ```
   Keep `sim:=false` (the default). The bridge prints `Sending map ... to the simulator on connection`.
2. On Windows, start the simulator. It connects to `127.0.0.1:4567` by itself (**Auto Connect**). Without Auto Connect, hover over the top of the window, click **Menu**, set **Autonomous** and click **Connect** once: **Connect** toggles, so a second click disconnects.
3. The simulator shows the walls of the map and the car at the start of the raceline. RViz opens with the stack (`rviz:=false` to skip it). To move the car or fix its position on the map, use **2D Pose Estimate**: it moves the car in the simulator and resets the localization to the same pose, as in the f1tenth_gym simulator (see the [`stack_master` README](../../stack_master/README.md#changing-initial-pose-after-mapping-for-cartographer)).
4. Drive with the joystick, as on the car, or start time trials in a second terminal:
   ```bash
   ros2 launch stack_master time_trials_launch.xml racecar_version:=NUCNUC
   ```

To change the map, stop the stack and launch it again with another `map_name`. The simulator can stay open.

Maps are made on the real car, not in the simulator. `mapping_launch.xml autodrive:=true` works, but sends no map to the simulator.


## Troubleshooting

- **The simulator does not connect (nothing on `/autodrive/roboracer_1/lidar`):** check that Windows reaches the bridge by opening `http://127.0.0.1:4567/socket.io/?EIO=3&transport=polling` in a browser on Windows, with the stack running. It should show `{"sid":...}`. If it does not load, enable host networking in Docker Desktop (**Settings → Resources → Network → Enable host networking**, then **Apply & restart**) and restart the race stack container. On Docker Desktop, `--network=host` otherwise means Docker's own VM, not Windows.
- **Port 4567 is in use:** a previous bridge is still running. It ignores a plain `pkill`, so use `pkill -9 -f autodrive_bridge`.
- **The simulator shows its built-in track:** the simulator has no map loader, or the map failed to build. With a build from the editor, the Unity Console shows an error starting with `MapLoader:`.
- **The stack stops at start-up with a map error:** the map folder could not be read. The bridge refuses to start, so that the stack and the simulator never run on different maps.
- **After hitting a wall, localization is lost:** the simulator respawns the car at the last checkpoint and the stack does not know it. Set the pose again with **2D Pose Estimate**.


## Build the Simulator

Only needed if the team build is not available. These steps run on Windows; Unity cannot run in the container. The AutoDRIVE Unity project is not part of this repository, because it contains Vehicle Physics Pro, which is licensed per person and may not be redistributed. Add the free Vehicle Physics Pro Community Edition to your own Unity account, and keep builds within the team.

1. **Get the project.** Install Unity 2022.3.52f1 (`unityhub://2022.3.52f1/1120fcb54228`). Clone the `AutoDRIVE-Simulator` branch of [Tinker-Twins/AutoDRIVE](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator) to a short path, then unzip its assets as its README says:
   ```bash
   git config --global core.longpaths true
   git clone --depth 1 --single-branch --branch AutoDRIVE-Simulator https://github.com/Tinker-Twins/AutoDRIVE.git C:/dev/AutoDRIVE
   cd C:/dev/AutoDRIVE && bash Tools/unzip-and-clean.sh && bash Tools/skip-zip-from-gitworktree.sh on
   ```
   Open it in Unity Hub (**Add project from disk**). The first import takes a long time.
2. **Copy the scripts.** Copy [unity/MapLoader.cs](unity/MapLoader.cs) and [unity/AutoConnect.cs](unity/AutoConnect.cs) to `Assets/Scripts/`. With VS Code attached to the container, right-click a file and choose **Download...**.
3. **Edit `Assets/Scripts/Socket.cs`** in three places:
   - After `private Quaternion CoSimRotation;`:
     ```csharp
     private MapLoader mapLoader; // Race stack `MapLoader` reference, if present in the scene
     ```
   - In `Start()`, after the line that caches the `SocketIOComponent`:
     ```csharp
     mapLoader = FindObjectOfType<MapLoader>(); // Cache `MapLoader`
     ```
   - In `OnBridge`, after `// Debug.Log(obj.data);`:
     ```csharp
     // Build the racetrack of the race stack map, if the bridge sent one (only in its first reply)
     JSONObject map = jsonObject.GetField("Map");
     if (map != null)
     {
         if (mapLoader != null) mapLoader.Load(map.str);
         else Debug.LogWarning("Socket: the bridge sent a map, but there is no `MapLoader` in the scene");
     }
     // Move the vehicle to the pose set with RViz's 2D Pose Estimate, if the bridge sent one
     JSONObject teleport = jsonObject.GetField("Teleport");
     if (teleport != null && mapLoader != null) mapLoader.Teleport(teleport.str);
     ```
4. **Edit `Assets/Scripts/ResetManager.cs`**, so that a reset returns to the start of the loaded track. Replace the line `void Start()` with the following; the existing body of `Start()` becomes the body of `StoreInitialStates()`:
   ```csharp
   void Start()
   {
       StoreInitialStates();
   }

   // Stores the current states as the initial states that a reset returns to
   public void StoreInitialStates()
   ```
5. **Add the components.** Open the scene `Assets/Scenes/RoboRacer - Sim Racing.unity` (not `- H2H`). Select **Reset Manager** in the Hierarchy and add **Map Loader** and **Auto Connect**. Drag `Assets/Materials/Air Duct/Air_Duct_Aluminium.mat` into **Wall Material** (without it the walls are magenta). Save the scene; the Console must show no red errors.
6. **Test in the editor.** Launch the stack as in [Every Run](#every-run) and press **Play**. The Console shows `MapLoader: loaded map Victoria`.
7. **Build.** In **File → Build Settings**, tick only `RoboRacer - Sim Racing` and untick `NeoRacer - AuE-X930`; select Windows, Intel 64-bit. In **Project Settings → Player → Other Settings**, set **Scripting Backend** to **Mono**. Build to a folder outside the project, for example `C:\dev\AutoDRIVE-builds\roboracer`.


## Reference

### `autodrive_roboracer`
Vendored copy of the AutoDRIVE devkit (ROS 2 bridge between the simulator and ROS).

- Upstream: `AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing`, release `2026-iros` (`autodrive_devkit.zip`, tag commit `639579a`)
- License: BSD 2-Clause, see [`autodrive_roboracer/LICENSE`](autodrive_roboracer/LICENSE) (taken from the upstream repository, the release zip ships without it)

Local changes:
- `autodrive_roboracer/autodrive_bridge.py`: the `broadcast_transforms(...)` call is commented out. Its `world -> roboracer_1 -> {imu, lidar, ...}` tree would give the `imu` frame a second parent next to the stack's `base_link -> imu`.
- `autodrive_roboracer/autodrive_bridge.py`: new optional parameter `map_path` (a folder in `stack_master/maps/`). If it is set, the bridge sends the map as an extra `Map` field in the first reply of each connection. `base_system_launch.xml` sets it from `map_name`.
- `autodrive_roboracer/autodrive_bridge.py`: with a map, it subscribes to `/initialpose` (RViz's 2D Pose Estimate) and sends the pose once as an extra `Teleport` field (`x,y,yaw` in the map frame), so that the simulator moves the car there.
- `autodrive_roboracer/map_to_sim.py`: new file, not part of upstream. It traces the walls from the map image (the free area that contains the start of `global_traj_wpnts_iqp`, the raceline the stack drives) and generates checkpoints along that raceline.

### `autodrive_adapter`
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

### `unity`
`MapLoader.cs` builds the map in the simulator: round duct walls (radius 0.15 m, thinner where the wall in the map is thin) on the layer the LiDAR hits, checkpoints and a finish line so that laps and wall respawns work, and the start pose. It hides the built-in tracks, and does nothing if the bridge sends no map. It needs exactly one car, so it does not work in the `- H2H` scene. `AutoConnect.cs` presses **Connect** when the simulator starts.

### Without the Stack
- Bridge only, with a map: `ros2 run autodrive_roboracer autodrive_bridge --ros-args -p map_path:=$HOME/ws/src/race_stack/stack_master/maps/Victoria`
- Drive manually: `ros2 run autodrive_roboracer teleop_keyboard`
- Linux simulator (released build, no map loader) in Docker on the host: `~/ws/src/race_stack/.docker_utils/autodrive_sim_dock.sh`
