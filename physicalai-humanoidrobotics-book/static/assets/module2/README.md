# Module 2 Assets

This directory contains static assets for Module 2: The Digital Twin

## Directory Structure

- **diagrams/**: Mermaid diagram source backups and exported PNGs
- **screenshots/**: Gazebo, Unity, and RViz2 screenshots
- **meshes/**: Sample URDF collision meshes (STL, OBJ, DAE)
- **videos/**: Tutorial videos and demonstration recordings

## Asset Guidelines

### Images
- Format: PNG or JPG
- Max size: 1MB per image
- Naming: `{chapter}-{description}-{number}.png`
  - Example: `ch1-gazebo-spawn-01.png`

### Videos
- Format: MP4 (H.264)
- Max size: 10MB per video
- Hosting: Upload to YouTube, embed link in MDX

### 3D Models
- Format: STL (preferred), OBJ, DAE
- Units: Meters (ROS 2 standard)
- Naming: `{robot_part}.stl`
  - Example: `robot_base.stl`

## Usage in MDX

### Images
```mdx
![Alt text](/assets/module2/screenshots/ch1-gazebo-spawn-01.png)
```

### Videos (YouTube embed)
```mdx
<iframe width="560" height="315"
  src="https://www.youtube.com/embed/VIDEO_ID"
  title="Video title"
  frameborder="0"
  allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
  allowfullscreen>
</iframe>
```

## Accessibility

All images MUST have meaningful alt text describing the content for screen readers.

❌ Bad: `![Image](/assets/module2/screenshots/image1.png)`
✅ Good: `![Gazebo GUI showing spawned robot with collision visualization](/assets/module2/screenshots/ch1-gazebo-spawn-01.png)`
