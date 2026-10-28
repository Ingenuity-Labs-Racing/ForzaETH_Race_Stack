# Map Loader for the AutoDRIVE Unity Project

## Documentation Overview

`MapLoader.cs` builds the racetrack of a race stack map inside the AutoDRIVE simulator at runtime, so that the simulator runs on the tracks in `stack_master/maps/`. See [autodrive_map_loader_plan.md](../../../docs/autodrive_map_loader_plan.md) for the design.

The bridge (`autodrive_roboracer/map_to_sim.py`) traces the walls and generates the checkpoints. It sends the result to the simulator in the first reply of each connection. `MapLoader.cs` then builds, in the `RoboRacer - Sim Racing` scene:
- round duct walls (one mesh with a collider per outline), on the `Default` layer that the LiDAR hits
- checkpoint triggers named `0`, `1`, ... with the `Checkpoint` tag and layer, and a finish line with the `Finish Line A` tag and layer, so that `LapTimer` counts laps and respawns the car at a checkpoint after hitting a wall
- the start pose of the car at the start of the raceline

The built-in tracks (the `Infrastructure` object) are hidden. Without a map from the bridge, nothing changes.

| Section | Description |
|---|---|
| [Setup](#setup) | Add the script to the Unity project. |
| [Test](#test) | Run it in the editor. |
| [Limits](#limits) | What has and has not been verified. |

The AutoDRIVE Unity project is not part of this repository (see the licensing rules in the plan), so these steps modify your local copy.


## Setup

Use the project from Step 1 of the plan (Unity 2022.3.52f1, `C:\dev\AutoDRIVE`).

1. **Copy the script.** Copy [MapLoader.cs](MapLoader.cs) to `Assets/Scripts/MapLoader.cs`. With VS Code attached to the container, right-click the file and choose **Download...**.
2. **Edit `Assets/Scripts/Socket.cs`** in three places:
   - Add a field next to the other private fields (after `private Quaternion CoSimRotation;`):
     ```csharp
     private MapLoader mapLoader; // Race stack `MapLoader` reference, if present in the scene
     ```
   - In `Start()`, after the line that caches the `SocketIOComponent`:
     ```csharp
     mapLoader = FindObjectOfType<MapLoader>(); // Cache `MapLoader`
     ```
   - In `OnBridge`, after the line `// Debug.Log(obj.data);`:
     ```csharp
     // Build the racetrack of the race stack map, if the bridge sent one (only in its first reply)
     JSONObject map = jsonObject.GetField("Map");
     if (map != null)
     {
         if (mapLoader != null) mapLoader.Load(map.str);
         else Debug.LogWarning("Socket: the bridge sent a map, but there is no `MapLoader` in the scene");
     }
     ```
     `Load` logs its own errors and never throws. That matters because `SocketIOComponent` swallows exceptions of event handlers, so an exception here would silently stop the telemetry.
3. **Edit `Assets/Scripts/ResetManager.cs`.** The reset must return to the start of the new track. Replace the line `void Start()` with:
   ```csharp
   void Start()
   {
       StoreInitialStates();
   }

   // Stores the current states as the initial states that a reset returns to
   public void StoreInitialStates()
   ```
   The existing body of `Start()` (the `{ ... }` that follows) is now the body of `StoreInitialStates()`. Nothing else changes.
4. **Add the component.** Open the scene `Assets/Scenes/RoboRacer - Sim Racing.unity` (the single-car scene, not `- H2H`). In the Hierarchy, select **Reset Manager** (an object that is active from the start; the `Socket` object is not) and choose **Add Component → Map Loader**.
   Optionally, to connect to the bridge as soon as the simulator starts, copy [AutoConnect.cs](AutoConnect.cs) to `Assets/Scripts/` as well and add **Auto Connect** to the same object. It does what the **Connect** button does, so the button and its label stay consistent. The driving mode already starts as `Autonomous`.
5. **Set the material.** In the Inspector, drag `Assets/Materials/Air Duct/Air_Duct_Aluminium.mat` into **Wall Material**. `Air_Duct_Black.mat` and `Air_Duct_White.mat` also work. Without a material the walls render magenta.
6. Save the scene (Ctrl+S). Wait for the script to compile. Check the Console for red errors and send them if there are any: the script has not been compiled yet.


## Test

1. Start the bridge with a stack map in the container. Without the full stack, the bridge alone is enough:
   ```bash
   ros2 run autodrive_roboracer autodrive_bridge --ros-args -p map_path:=$HOME/ws/src/race_stack/stack_master/maps/Victoria
   ```
   With the stack, `map_name` does the same:
   ```bash
   ros2 launch stack_master base_system_launch.xml autodrive:=true map_name:=Victoria
   ```
   The bridge prints `Sending map ... to the simulator on connection`. If the map cannot be read, the bridge stops with an error, so that the stack and the simulator never run on different maps.
2. In Unity, open the scene and press **Play**. Hover over the top of the Game view, click **Menu**, set **Autonomous** and click **Connect**.
3. The Console shows `MapLoader: loaded map Victoria`. The built-in track disappears, the duct walls of Victoria appear and the car is at the start of the raceline. If the map cannot be built, the Console shows an error starting with `MapLoader:` (or an exception), and the built-in track stays.
4. Check, in this order:
   - **Alignment:** `ros2 topic echo --once /autodrive/roboracer_1/ips` is the start of the raceline in `global_waypoints.json` (`global_traj_wpnts_iqp`, point 0, `x_m` and `y_m`).
   - **LiDAR:** `ros2 topic echo --once /autodrive/roboracer_1/lidar` has ranges that match the distance to the walls. With the stack running, `/scan` lines up with the stack's map in RViz.
   - **Walls and respawn:** drive into a wall (for example with `ros2 run autodrive_roboracer teleop_keyboard`). `/autodrive/roboracer_1/collision_count` increases and the car respawns on the raceline.
   - **Laps:** drive a lap. `/autodrive/roboracer_1/lap_count` increases when the car crosses the start line after passing all checkpoints.
5. Disconnect and connect again. The track is built again.


## Limits

- **Verified outside Unity:** the geometry in `map_to_sim.py` on all eight maps in `stack_master/maps/` (the duct surface at LiDAR height lies on the map's wall edge within 1 cm on average and 5 cm at most; the checkpoints span the track wall to wall) and the bridge (the map is sent once per connection and again after a reconnect; nothing is sent without `map_path`).
- **Not verified:** `MapLoader.cs` itself. It has not been compiled or run, because Unity cannot run in the container. Expect to fix small mistakes on first import.
- **Walls:** thin infield walls (about 0.2 m in `GLC_smile_small` and `glc_ot_ez`) get thinner ducts, down to a radius of about 0.09 m. Every duct is taller than the LiDAR (0.155 m above the ground, measured on the simulator). The ducts have both windings in their meshes, so the LiDAR hits them from the track side whatever the direction of the outline.
- **Track camera:** the cameras that followed the built-in track (`FollowTarget`, used by `Trackcam`) are pointed at the center of the loaded track. Cameras that follow the car are unchanged.
- **Single car:** the loader refuses to load (with a Console error) unless the scene has exactly one car with a `LapTimer`, so it does not work in the `- H2H` scene.
- **Respawn height:** after hitting a wall, `LapTimer` places the car at the height of the checkpoint (0.01 m), as on the built-in tracks.
- **Switching maps** needs a bridge restart. The simulator replaces the track whenever it receives a map.
