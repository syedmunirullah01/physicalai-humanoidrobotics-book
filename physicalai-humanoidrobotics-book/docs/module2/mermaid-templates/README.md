# Mermaid Diagram Templates for Module 2

Reusable Mermaid diagram snippets for consistency across chapters.

## Available Templates

### 1. ROS 2 Computation Graph
**Use for**: Showing nodes, topics, and data flow

### 2. TF Tree
**Use for**: Coordinate frame hierarchies

### 3. Sensor Data Flow (Sequence)
**Use for**: Gazebo → Plugin → ROS 2 → RViz2 pipeline

### 4. Unity ROS Integration
**Use for**: Bidirectional Unity-ROS communication

## Usage

1. Copy template code from corresponding `.md` file
2. Customize node labels, topic names, and relationships
3. Add text alternative in `<details>` block
4. Test rendering in Docusaurus preview

## Color Coding

- **Nodes**: Blue (`#4A90E2`)
- **Topics**: Green (`#7ED321`)
- **Services**: Orange (`#F5A623`)
- **Sensors**: Teal (`#50E3C2`)
- **Errors**: Red (`#D0021B`)

## Accessibility Checklist

- [ ] Preceding paragraph explains diagram purpose
- [ ] Semantic labels (not generic node1, node2)
- [ ] Text alternative in `<details>` block
- [ ] Tested on mobile (responsive)
