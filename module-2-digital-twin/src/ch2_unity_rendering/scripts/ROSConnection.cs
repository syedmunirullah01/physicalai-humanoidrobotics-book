using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

/// <summary>
/// Establishes connection to ROS-TCP-Endpoint running on ROS 2 machine
///
/// Usage:
/// 1. Attach to empty GameObject named "ROSManager"
/// 2. Configure hostName and port in Inspector
/// 3. Play scene - connection established automatically
/// </summary>
public class ROSConnection : MonoBehaviour
{
    [Header("ROS Connection Settings")]
    [Tooltip("IP address of ROS 2 machine (localhost for same machine, or remote IP)")]
    public string hostName = "localhost";

    [Tooltip("TCP port for ROS-TCP-Endpoint (default: 10000)")]
    public int port = 10000;

    [Header("Connection Status")]
    [SerializeField] private bool isConnected = false;

    private ROSConnection rosConnection;

    void Start()
    {
        ConnectToROS();
    }

    void ConnectToROS()
    {
        try
        {
            // Get or create singleton ROS connection instance
            rosConnection = ROSConnection.GetOrCreateInstance();

            // Connect to ROS-TCP-Endpoint
            rosConnection.Connect(hostName, port);

            isConnected = true;
            Debug.Log($"[ROSConnection] Successfully connected to ROS 2 at {hostName}:{port}");
        }
        catch (System.Exception e)
        {
            isConnected = false;
            Debug.LogError($"[ROSConnection] Failed to connect to ROS 2: {e.Message}");
            Debug.LogError($"[ROSConnection] Ensure ROS-TCP-Endpoint is running: ros2 run ros_tcp_endpoint default_server_endpoint");
        }
    }

    void OnApplicationQuit()
    {
        if (isConnected)
        {
            Debug.Log("[ROSConnection] Disconnecting from ROS 2...");
            // Connection cleanup handled by ROS-TCP-Connector
        }
    }

    /// <summary>
    /// Check if currently connected to ROS
    /// </summary>
    public bool IsConnected()
    {
        return isConnected;
    }
}
