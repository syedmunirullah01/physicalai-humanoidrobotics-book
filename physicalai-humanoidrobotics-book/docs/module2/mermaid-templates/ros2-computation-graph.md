# ROS 2 Computation Graph Template

## Template Code

```mermaid
graph TD
    Node1["/node_name_1<br/>(package_name)"]
    Node2["/node_name_2<br/>(package_name)"]
    Topic1(["/topic_name<br/>(message_type)"])

    Node1 -->|publishes| Topic1
    Topic1 -->|subscribes| Node2

    style Node1 fill:#4A90E2,stroke:#2E5C8A,color:#fff
    style Node2 fill:#4A90E2,stroke:#2E5C8A,color:#fff
    style Topic1 fill:#7ED321,stroke:#5FA319,color:#000
```

## Customization Guide

1. **Node Names**: Replace `node_name_1`, `node_name_2` with actual node names (e.g., `/gazebo`, `/robot_state_publisher`)
2. **Package Names**: Add package in parentheses (e.g., `(gazebo_ros)`)
3. **Topics**: Use rounded boxes `([...])` for topics
4. **Message Types**: Include message type (e.g., `(geometry_msgs/Twist)`)

## Example: TurtleBot3 Navigation

```mermaid
graph TD
    Gazebo["/gazebo<br/>(gazebo_ros)"]
    RSP["/robot_state_publisher<br/>(robot_state_publisher)"]
    Nav["/navigation_node<br/>(nav2)"]

    LidarTopic(["/scan<br/>(sensor_msgs/LaserScan)"])
    CmdVelTopic(["/cmd_vel<br/>(geometry_msgs/Twist)"])

    Gazebo -->|publishes| LidarTopic
    LidarTopic -->|subscribes| Nav
    Nav -->|publishes| CmdVelTopic
    CmdVelTopic -->|subscribes| Gazebo

    style Gazebo fill:#4A90E2,stroke:#2E5C8A,color:#fff
    style RSP fill:#4A90E2,stroke:#2E5C8A,color:#fff
    style Nav fill:#4A90E2,stroke:#2E5C8A,color:#fff
    style LidarTopic fill:#7ED321,stroke:#5FA319,color:#000
    style CmdVelTopic fill:#7ED321,stroke:#5FA319,color:#000
```

## Text Alternative Template

```markdown
<details>
<summary>Text alternative for ROS 2 Computation Graph</summary>

This diagram shows the ROS 2 nodes and topics for [system description]:
- **/node_name_1** (from package_name) publishes to **/topic_name** (message_type)
- **/node_name_2** (from package_name) subscribes to **/topic_name**

Data flows from node_name_1 through the topic to node_name_2.
</details>
```
