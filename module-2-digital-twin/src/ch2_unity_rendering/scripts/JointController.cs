using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;

/// <summary>
/// Subscribes to ROS /joint_states topic and updates Unity ArticulationBodies
///
/// Usage:
/// 1. Attach to robot root GameObject
/// 2. Assign all ArticulationBody components in joints array (Inspector)
/// 3. Play scene - joints will animate based on ROS 2 messages
///
/// ROS Topic: /joint_states (sensor_msgs/JointState)
/// </summary>
public class JointController : MonoBehaviour
{
    [Header("Robot Joints")]
    [Tooltip("Array of ArticulationBody components (Unity's joint representation)")]
    public ArticulationBody[] joints;

    [Header("ROS Topic")]
    [Tooltip("ROS topic to subscribe to for joint states")]
    public string jointStateTopic = "/joint_states";

    [Header("Debug")]
    [SerializeField] private bool showDebugLogs = false;

    private ROSConnection ros;
    private Dictionary<string, ArticulationBody> jointMap;

    void Start()
    {
        // Get ROS connection instance
        ros = ROSConnection.GetOrCreateInstance();

        // Build joint name -> ArticulationBody mapping
        BuildJointMap();

        // Subscribe to joint states topic
        ros.Subscribe<JointStateMsg>(jointStateTopic, UpdateJointStates);

        Debug.Log($"[JointController] Subscribed to {jointStateTopic}");
        Debug.Log($"[JointController] Tracking {joints.Length} joints");
    }

    void BuildJointMap()
    {
        jointMap = new Dictionary<string, ArticulationBody>();

        foreach (var joint in joints)
        {
            if (joint != null)
            {
                // Use GameObject name as joint identifier (must match URDF joint name)
                string jointName = joint.gameObject.name;
                jointMap[jointName] = joint;

                if (showDebugLogs)
                {
                    Debug.Log($"[JointController] Registered joint: {jointName}");
                }
            }
        }
    }

    void UpdateJointStates(JointStateMsg msg)
    {
        // Iterate through all joints in ROS message
        for (int i = 0; i < msg.name.Length; i++)
        {
            string jointName = msg.name[i];

            // Check if we have a corresponding Unity joint
            if (jointMap.ContainsKey(jointName))
            {
                ArticulationBody joint = jointMap[jointName];

                // Get position from ROS (in radians)
                float positionRad = (float)msg.position[i];

                // Convert radians to degrees (Unity uses degrees)
                float positionDeg = positionRad * Mathf.Rad2Deg;

                // Update ArticulationBody target
                UpdateJointTarget(joint, positionDeg);

                if (showDebugLogs)
                {
                    Debug.Log($"[JointController] {jointName}: {positionDeg:F2}°");
                }
            }
            else if (showDebugLogs)
            {
                Debug.LogWarning($"[JointController] Unknown joint in ROS message: {jointName}");
            }
        }
    }

    void UpdateJointTarget(ArticulationBody joint, float targetDegrees)
    {
        // Check joint type (revolute, prismatic, etc.)
        if (joint.jointType == ArticulationJointType.RevoluteJoint)
        {
            // Update revolute joint target position
            var drive = joint.xDrive;
            drive.target = targetDegrees;
            joint.xDrive = drive;
        }
        else if (joint.jointType == ArticulationJointType.PrismaticJoint)
        {
            // For prismatic joints, position is in meters (not degrees)
            var drive = joint.xDrive;
            drive.target = targetDegrees / Mathf.Rad2Deg;  // Convert back to meters
            joint.xDrive = drive;
        }
    }

    void OnValidate()
    {
        // Validate joints array is populated
        if (joints == null || joints.Length == 0)
        {
            Debug.LogWarning("[JointController] Joints array is empty! Assign ArticulationBody components in Inspector.");
        }
    }
}
