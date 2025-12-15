using UnityEngine;
using UnityEngine.AI;

/// <summary>
/// Autonomous navigation behavior for virtual humans using Unity NavMeshAgent
///
/// Usage:
/// 1. Attach to virtual human GameObject
/// 2. Add NavMeshAgent component (Component → Navigation → Nav Mesh Agent)
/// 3. Add Animator component with "Speed" float parameter
/// 4. Create empty GameObjects as waypoints in scene
/// 5. Assign waypoints array in Inspector
/// 6. Bake NavMesh (Window → AI → Navigation → Bake)
///
/// Behavior:
/// - Patrols between waypoints in sequence
/// - Updates animation based on movement speed
/// - Avoids obstacles automatically via NavMesh
/// </summary>
[RequireComponent(typeof(NavMeshAgent))]
[RequireComponent(typeof(Animator))]
public class HumanBehavior : MonoBehaviour
{
    [Header("Waypoints")]
    [Tooltip("Array of Transform positions to patrol between")]
    public Transform[] waypoints;

    [Tooltip("Wait time at each waypoint (seconds)")]
    [Range(0f, 10f)]
    public float waypointWaitTime = 2f;

    [Header("Movement")]
    [Tooltip("Walking speed (m/s) - also adjusts NavMeshAgent speed")]
    [Range(0.5f, 3.0f)]
    public float walkSpeed = 1.4f;

    [Tooltip("Rotation speed when turning to face waypoint")]
    [Range(50f, 500f)]
    public float rotationSpeed = 200f;

    [Header("Animation")]
    [Tooltip("Animator parameter name for movement speed")]
    public string speedParameterName = "Speed";

    [Tooltip("Speed multiplier for animation (tune to match walk cycle)")]
    [Range(0.1f, 2.0f)]
    public float animationSpeedMultiplier = 1.0f;

    private NavMeshAgent agent;
    private Animator animator;
    private int currentWaypointIndex = 0;
    private float waypointArrivalTime = 0f;
    private bool waitingAtWaypoint = false;

    void Start()
    {
        // Get required components
        agent = GetComponent<NavMeshAgent>();
        animator = GetComponent<Animator>();

        // Configure NavMeshAgent
        agent.speed = walkSpeed;
        agent.angularSpeed = rotationSpeed;
        agent.acceleration = 8f;  // Smooth acceleration
        agent.stoppingDistance = 0.5f;  // Stop 0.5m from waypoint

        // Validate setup
        if (waypoints == null || waypoints.Length == 0)
        {
            Debug.LogWarning($"[HumanBehavior] {gameObject.name} has no waypoints assigned!");
            enabled = false;
            return;
        }

        // Start navigation
        GoToNextWaypoint();
    }

    void Update()
    {
        UpdateAnimation();
        CheckWaypointArrival();
    }

    void UpdateAnimation()
    {
        // Get current velocity magnitude
        float speed = agent.velocity.magnitude;

        // Normalize speed (0 = idle, 1 = walking)
        float normalizedSpeed = speed / walkSpeed * animationSpeedMultiplier;

        // Update animator parameter
        if (animator != null && animator.parameterCount > 0)
        {
            animator.SetFloat(speedParameterName, normalizedSpeed);
        }
    }

    void CheckWaypointArrival()
    {
        // Skip if no current destination
        if (agent.pathPending || waypoints.Length == 0)
            return;

        // Check if arrived at waypoint
        if (!waitingAtWaypoint && agent.remainingDistance <= agent.stoppingDistance)
        {
            // Arrived - start waiting
            waitingAtWaypoint = true;
            waypointArrivalTime = Time.time;

            OnWaypointReached();
        }

        // Check if finished waiting
        if (waitingAtWaypoint && Time.time - waypointArrivalTime >= waypointWaitTime)
        {
            waitingAtWaypoint = false;
            GoToNextWaypoint();
        }
    }

    void GoToNextWaypoint()
    {
        if (waypoints.Length == 0) return;

        // Get next waypoint
        Transform targetWaypoint = waypoints[currentWaypointIndex];

        // Set NavMeshAgent destination
        agent.SetDestination(targetWaypoint.position);

        // Increment index (loop back to 0 after last waypoint)
        currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Length;

        Debug.Log($"[HumanBehavior] {gameObject.name} navigating to waypoint {currentWaypointIndex}");
    }

    void OnWaypointReached()
    {
        // Optional: Play idle animation, look around, etc.
        Debug.Log($"[HumanBehavior] {gameObject.name} reached waypoint, waiting {waypointWaitTime}s");
    }

    void OnDrawGizmos()
    {
        // Draw waypoint connections in Scene view
        if (waypoints != null && waypoints.Length > 1)
        {
            Gizmos.color = Color.yellow;

            for (int i = 0; i < waypoints.Length; i++)
            {
                if (waypoints[i] == null) continue;

                // Draw waypoint sphere
                Gizmos.DrawWireSphere(waypoints[i].position, 0.3f);

                // Draw line to next waypoint
                int nextIndex = (i + 1) % waypoints.Length;
                if (waypoints[nextIndex] != null)
                {
                    Gizmos.DrawLine(waypoints[i].position, waypoints[nextIndex].position);
                }
            }
        }

        // Draw current navigation path
        if (Application.isPlaying && agent != null && agent.hasPath)
        {
            Gizmos.color = Color.green;
            Vector3[] corners = agent.path.corners;

            for (int i = 0; i < corners.Length - 1; i++)
            {
                Gizmos.DrawLine(corners[i], corners[i + 1]);
            }
        }
    }

    void OnValidate()
    {
        // Update NavMeshAgent speed when walkSpeed changes in Inspector
        if (agent != null)
        {
            agent.speed = walkSpeed;
        }
    }
}
