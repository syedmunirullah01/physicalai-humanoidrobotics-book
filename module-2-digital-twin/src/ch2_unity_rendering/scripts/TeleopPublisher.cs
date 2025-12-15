using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

/// <summary>
/// Publishes keyboard input to ROS /cmd_vel topic for robot teleoperation
///
/// Usage:
/// 1. Attach to any GameObject (e.g., ROSManager)
/// 2. Configure topic name and velocity limits in Inspector
/// 3. Play scene and use WASD keys to control robot
///
/// Controls:
/// - W/S: Forward/Backward
/// - A/D: Turn Left/Right
/// - Shift: Boost speed
///
/// ROS Topic: /cmd_vel (geometry_msgs/Twist)
/// </summary>
public class TeleopPublisher : MonoBehaviour
{
    [Header("ROS Topic")]
    [Tooltip("ROS topic to publish velocity commands to")]
    public string topicName = "/cmd_vel";

    [Header("Velocity Limits")]
    [Tooltip("Maximum linear velocity (m/s)")]
    [Range(0.1f, 2.0f)]
    public float maxLinearVelocity = 0.5f;

    [Tooltip("Maximum angular velocity (rad/s)")]
    [Range(0.1f, 3.0f)]
    public float maxAngularVelocity = 1.0f;

    [Tooltip("Speed boost multiplier when holding Shift")]
    [Range(1.0f, 3.0f)]
    public float boostMultiplier = 2.0f;

    [Header("Smoothing")]
    [Tooltip("Acceleration smoothing factor (0=instant, 1=very smooth)")]
    [Range(0f, 0.9f)]
    public float smoothing = 0.7f;

    [Header("Debug")]
    [SerializeField] private bool showDebugGUI = true;

    private ROSConnection ros;
    private float currentLinear = 0f;
    private float currentAngular = 0f;

    void Start()
    {
        // Get ROS connection instance
        ros = ROSConnection.GetOrCreateInstance();

        // Register as publisher
        ros.RegisterPublisher<TwistMsg>(topicName);

        Debug.Log($"[TeleopPublisher] Publishing to {topicName}");
        Debug.Log("[TeleopPublisher] Controls: WASD = move, Shift = boost");
    }

    void Update()
    {
        // Read keyboard input
        float targetLinear = 0f;
        float targetAngular = 0f;

        // Linear velocity (forward/backward)
        if (Input.GetKey(KeyCode.W)) targetLinear = maxLinearVelocity;
        if (Input.GetKey(KeyCode.S)) targetLinear = -maxLinearVelocity;

        // Angular velocity (turn left/right)
        if (Input.GetKey(KeyCode.A)) targetAngular = maxAngularVelocity;
        if (Input.GetKey(KeyCode.D)) targetAngular = -maxAngularVelocity;

        // Boost mode (Shift key)
        if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
        {
            targetLinear *= boostMultiplier;
            targetAngular *= boostMultiplier;
        }

        // Apply smoothing (lerp towards target)
        currentLinear = Mathf.Lerp(currentLinear, targetLinear, 1f - smoothing);
        currentAngular = Mathf.Lerp(currentAngular, targetAngular, 1f - smoothing);

        // Create Twist message
        TwistMsg msg = new TwistMsg
        {
            linear = new Vector3Msg
            {
                x = currentLinear,
                y = 0,
                z = 0
            },
            angular = new Vector3Msg
            {
                x = 0,
                y = 0,
                z = currentAngular
            }
        };

        // Publish to ROS
        ros.Publish(topicName, msg);
    }

    void OnGUI()
    {
        if (showDebugGUI)
        {
            // Display teleoperation status in top-left corner
            GUI.Box(new Rect(10, 10, 250, 100), "Teleoperation");

            GUI.Label(new Rect(20, 35, 230, 20), $"Linear: {currentLinear:F2} m/s");
            GUI.Label(new Rect(20, 55, 230, 20), $"Angular: {currentAngular:F2} rad/s");
            GUI.Label(new Rect(20, 75, 230, 20), $"Topic: {topicName}");
        }
    }

    void OnDrawGizmos()
    {
        // Draw velocity vector in Scene view
        if (Application.isPlaying && currentLinear != 0)
        {
            Vector3 velocityVector = transform.forward * currentLinear * 2f;
            Gizmos.color = Color.green;
            Gizmos.DrawLine(transform.position, transform.position + velocityVector);
            Gizmos.DrawSphere(transform.position + velocityVector, 0.1f);
        }
    }
}
