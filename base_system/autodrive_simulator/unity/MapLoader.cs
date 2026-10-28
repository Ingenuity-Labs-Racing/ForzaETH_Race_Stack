using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

public class MapLoader : MonoBehaviour
{
    /*
    This script builds a racetrack at runtime from the geometry that the race stack bridge sends in the `Map`
    field of its first reply (base64 encoded JSON, see `map_to_sim.py`): round duct walls, checkpoints, the finish
    line and the start pose of the vehicle. The built-in tracks are hidden.

    The geometry arrives in the ROS map frame and is converted to Unity like `GPS.cs` does,
    Unity (x, y, z) = (-y, z, x). A ROS yaw of `a` is a rotation of `-a` about the vertical axis.
    */

    public Material WallMaterial; // Material of the duct walls
    public string BuiltInTracks = "Infrastructure"; // Root object of the built-in tracks, hidden when a map is loaded
    public int DuctSides = 12; // Number of sides of the duct walls
    public float TriggerHeight = 0.254f; // Height of the checkpoint triggers
    public float TriggerThickness = 0.2f; // Thickness of the checkpoint triggers along the driving direction
    public float TriggerY = 0.01f; // Height of the center of the checkpoint triggers

    private const string RacetrackName = "Stack Map Walls"; // `LapTimer` respawns the vehicle on hitting colliders of this name
    private GameObject Racetrack; // Generated racetrack
    private GameObject BuiltInRoot; // Found on the first load, `GameObject.Find` does not find it once it is hidden
    private float VehicleHeight = float.NaN; // Height at which the vehicle is placed, taken from the scene

    [Serializable] public class Wall { public float[] points; } // x, y and duct radius of each point of a closed outline
    [Serializable] public class Placement { public float x; public float y; public float yaw; public float width; }
    [Serializable] public class MapData
    {
        public string name;
        public List<Wall> walls;
        public Placement spawn;
        public Placement finish;
        public List<Placement> checkpoints; // Checkpoint `i` is named "i", checkpoint 0 is at the finish line
    }

    // Build the racetrack from the base64 encoded JSON map sent by the bridge.
    // `SocketIOComponent` swallows exceptions of event handlers, and an exception here would stop `Socket.OnBridge`
    // before it sends the next telemetry, so errors are logged here and leave the scene as it was.
    public void Load(string encodedMap)
    {
        GameObject racetrack = null;
        try
        {
            MapData map = JsonUtility.FromJson<MapData>(Encoding.UTF8.GetString(Convert.FromBase64String(encodedMap)));
            LapTimer[] lapTimers = FindObjectsOfType<LapTimer>();
            LapTimer lapTimer = lapTimers.Length == 1 ? lapTimers[0] : null; // Single-car scenes only
            Rigidbody vehicleRigidBody = lapTimer != null ? lapTimer.GetComponent<Rigidbody>() : null;
            string error = Validate(map, vehicleRigidBody);
            if (error != null)
            {
                Debug.LogError("MapLoader: " + error);
                return;
            }

            // Build the new racetrack before changing the scene
            racetrack = new GameObject("Stack Map " + map.name);
            Bounds bounds = new Bounds(ToUnity(map.spawn.x, map.spawn.y, 0f), Vector3.zero);
            foreach (Wall wall in map.walls) bounds.Encapsulate(BuildWall(racetrack.transform, wall).GetComponent<MeshRenderer>().bounds);
            Transform[] checkpoints = new Transform[map.checkpoints.Count];
            for (int i = 0; i < checkpoints.Length; i++) checkpoints[i] = BuildTrigger(racetrack.transform, map.checkpoints[i], i.ToString(), "Checkpoint").transform;
            BuildTrigger(racetrack.transform, map.finish, "Finish Line", "Finish Line A");
            GameObject cameraTarget = new GameObject("Camera Target"); // Center of the racetrack, for the track cameras
            cameraTarget.transform.SetParent(racetrack.transform, false);
            cameraTarget.transform.position = new Vector3(bounds.center.x, 0f, bounds.center.z);

            // Replace the built-in or previous racetrack
            if (BuiltInRoot == null) BuiltInRoot = GameObject.Find(BuiltInTracks);
            RetargetCameras(cameraTarget);
            if (BuiltInRoot != null) BuiltInRoot.SetActive(false);
            if (Racetrack != null) DestroyRacetrack(Racetrack);
            Racetrack = racetrack;
            PlaceVehicle(map, lapTimer, vehicleRigidBody, checkpoints);
            Debug.Log("MapLoader: loaded map " + map.name);
        }
        catch (Exception e)
        {
            if (racetrack != null && racetrack != Racetrack) DestroyRacetrack(racetrack);
            Debug.LogException(e);
        }
    }

    // Destroy a generated racetrack. Its meshes are not destroyed with their game objects.
    private static void DestroyRacetrack(GameObject racetrack)
    {
        foreach (MeshFilter meshFilter in racetrack.GetComponentsInChildren<MeshFilter>(true)) Destroy(meshFilter.sharedMesh);
        Destroy(racetrack);
    }

    // Point the cameras that follow the built-in or the previous racetrack (`FollowTarget`, used by `Trackcam`) at the new one.
    // Cameras that follow the vehicle are left alone.
    private void RetargetCameras(GameObject cameraTarget)
    {
        foreach (FollowTarget follower in FindObjectsOfType<FollowTarget>(true))
            if (follower.Target != null && (IsPartOf(follower.Target, BuiltInRoot) || IsPartOf(follower.Target, Racetrack)))
                follower.Target = cameraTarget;
    }

    private static bool IsPartOf(GameObject child, GameObject root)
    {
        return root != null && child.transform.IsChildOf(root.transform);
    }

    // Return why the map cannot be loaded, or null if it can
    private string Validate(MapData map, Rigidbody vehicleRigidBody)
    {
        if (map == null || map.walls == null || map.walls.Count == 0 || map.checkpoints == null || map.checkpoints.Count == 0 || map.spawn == null || map.finish == null)
            return "incomplete map";
        foreach (Wall wall in map.walls)
            if (wall.points == null || wall.points.Length < 9 || wall.points.Length % 3 != 0) return "invalid wall outline in map " + map.name;
        if (vehicleRigidBody == null) return "expected exactly one vehicle with a `LapTimer` and a `Rigidbody` in the scene";
        if (LayerMask.NameToLayer("Checkpoint") < 0 || LayerMask.NameToLayer("Finish Line A") < 0) return "the `Checkpoint` or `Finish Line A` layer is missing";
        return null;
    }

    // Place the vehicle at the start of the raceline and reset the lap data for the new racetrack
    private void PlaceVehicle(MapData map, LapTimer lapTimer, Rigidbody vehicleRigidBody, Transform[] checkpoints)
    {
        if (float.IsNaN(VehicleHeight)) VehicleHeight = lapTimer.transform.position.y;
        vehicleRigidBody.velocity = Vector3.zero;
        vehicleRigidBody.angularVelocity = Vector3.zero;
        lapTimer.transform.position = ToUnity(map.spawn.x, map.spawn.y, VehicleHeight);
        lapTimer.transform.rotation = Quaternion.Euler(0f, -map.spawn.yaw * Mathf.Rad2Deg, 0f);

        lapTimer.RacetrackName = RacetrackName;
        lapTimer.Checkpoints = checkpoints;
        lapTimer.CurrentCheckpoint = 0;
        lapTimer.PreviousCheckpoint = 0;
        lapTimer.CheckpointCount = 0;
        lapTimer.CollisionCount = 0;
        lapTimer.LapCount = 0;
        lapTimer.LapTime = 0f;
        lapTimer.LastLapTime = Mathf.Infinity;
        lapTimer.BestLapTime = Mathf.Infinity;
        ResetManager resetManager = FindObjectOfType<ResetManager>();
        if (resetManager != null) resetManager.StoreInitialStates(); // A reset returns to the start of the raceline
    }

    // ROS map frame (x, y, z) to Unity world frame
    private static Vector3 ToUnity(float x, float y, float z)
    {
        return new Vector3(-y, z, x);
    }

    // Duct wall along a closed outline
    private GameObject BuildWall(Transform parent, Wall wall)
    {
        GameObject duct = new GameObject(RacetrackName); // On the `Default` layer, which the LiDAR hits
        duct.transform.SetParent(parent, false);
        Mesh mesh = DuctMesh(wall.points);
        duct.AddComponent<MeshFilter>().sharedMesh = mesh;
        duct.AddComponent<MeshRenderer>().sharedMaterial = WallMaterial;
        duct.AddComponent<MeshCollider>().sharedMesh = mesh;
        return duct;
    }

    // Checkpoint or finish line: a box across the track, whose forward axis is the driving direction.
    // `LapTimer` respawns the vehicle at the position and rotation of a checkpoint. The tag and layer of both are named the same.
    private GameObject BuildTrigger(Transform parent, Placement placement, string triggerName, string tagAndLayer)
    {
        GameObject trigger = new GameObject(triggerName);
        trigger.transform.SetParent(parent, false);
        trigger.tag = tagAndLayer;
        trigger.layer = LayerMask.NameToLayer(tagAndLayer); // Not the `Default` layer, so that the LiDAR does not hit it
        trigger.transform.position = ToUnity(placement.x, placement.y, TriggerY);
        trigger.transform.rotation = Quaternion.Euler(0f, -placement.yaw * Mathf.Rad2Deg, 0f);
        trigger.transform.localScale = new Vector3(placement.width, TriggerHeight, TriggerThickness);
        trigger.AddComponent<BoxCollider>().isTrigger = true;
        return trigger;
    }

    // Round duct lying on the ground, swept along a closed outline of (x, y, radius) points.
    // The mesh has both windings: raycasts (the LiDAR) only hit front faces, and where the outline bends more sharply
    // than the duct is wide, some faces turn inside out.
    private Mesh DuctMesh(float[] points)
    {
        int count = points.Length / 3;
        int ringCount = count * DuctSides;
        Vector3[] centers = new Vector3[count];
        for (int i = 0; i < count; i++) centers[i] = ToUnity(points[3 * i], points[3 * i + 1], points[3 * i + 2]); // Center at the height of the radius

        Vector3[] vertices = new Vector3[2 * ringCount]; // Second half: the same vertices for the back faces
        Vector3[] normals = new Vector3[2 * ringCount];
        Vector2[] uvs = new Vector2[2 * ringCount];
        float length = 0f;
        for (int i = 0; i < count; i++)
        {
            Vector3 tangent = centers[(i + 1) % count] - centers[(i + count - 1) % count];
            tangent.y = 0f;
            tangent.Normalize();
            Vector3 side = Vector3.Cross(Vector3.up, tangent);
            if (i > 0)
            {
                Vector3 step = centers[i] - centers[i - 1];
                step.y = 0f;
                length += step.magnitude;
            }
            for (int j = 0; j < DuctSides; j++)
            {
                float angle = 2f * Mathf.PI * j / DuctSides;
                Vector3 normal = Mathf.Cos(angle) * side + Mathf.Sin(angle) * Vector3.up;
                int v = i * DuctSides + j;
                vertices[v] = vertices[ringCount + v] = centers[i] + points[3 * i + 2] * normal;
                normals[v] = normal;
                normals[ringCount + v] = -normal;
                uvs[v] = uvs[ringCount + v] = new Vector2(length, (float)j / DuctSides);
            }
        }

        int[] triangles = new int[count * DuctSides * 12];
        int t = 0;
        for (int i = 0; i < count; i++)
        {
            int next = (i + 1) % count;
            for (int j = 0; j < DuctSides; j++)
            {
                int a = i * DuctSides + j;
                int b = i * DuctSides + (j + 1) % DuctSides;
                int c = next * DuctSides + j;
                int d = next * DuctSides + (j + 1) % DuctSides;
                triangles[t++] = a; triangles[t++] = c; triangles[t++] = b;
                triangles[t++] = b; triangles[t++] = c; triangles[t++] = d;
                triangles[t++] = ringCount + a; triangles[t++] = ringCount + b; triangles[t++] = ringCount + c;
                triangles[t++] = ringCount + b; triangles[t++] = ringCount + d; triangles[t++] = ringCount + c;
            }
        }

        Mesh mesh = new Mesh();
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        mesh.vertices = vertices;
        mesh.normals = normals;
        mesh.uv = uvs;
        mesh.triangles = triangles;
        mesh.RecalculateBounds();
        return mesh;
    }
}
