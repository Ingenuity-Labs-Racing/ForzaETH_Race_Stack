using UnityEngine;

public class AutoConnect : MonoBehaviour
{
    /*
    This script connects the simulator to the bridge when it starts, as if the Connect button was pressed. The address
    is the one in the menu (127.0.0.1:4567 by default, or the `-ip` and `-port` start-up arguments).
    */

    void Start()
    {
        SocketConnection socketConnection = FindObjectOfType<SocketConnection>(true); // On the `Socket` object, which is inactive until connected
        if (socketConnection != null) socketConnection.ToggleSocketConnection();
        else Debug.LogWarning("AutoConnect: no `SocketConnection` in the scene");
    }
}
