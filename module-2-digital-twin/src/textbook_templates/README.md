# Textbook Templates

**Purpose**: Reusable templates for ROS 2 packages, launch files, and URDF models

This directory contains template files that students can use as starting points for their own projects. All templates follow ROS 2 best practices and include detailed comments explaining each section.

## Available Templates

### ROS 2 Package Templates

**package.xml.template** - Standard ROS 2 package manifest
- Replace `PACKAGE_NAME` with your package name
- Uncomment dependencies as needed
- Includes common dependencies for ROS 2, Gazebo, and sensors

**setup.py.template** - Python package setup for ROS 2
- Configure for Python-based ROS 2 packages
- Includes entry points for nodes

### Launch File Templates

**minimal.launch.py.template** - Basic ROS 2 launch file
- Single node launcher
- Configurable parameters
- Example parameter passing

**gazebo_spawn.launch.py.template** - Gazebo robot spawning
- Launches Gazebo simulator
- Spawns robot from URDF
- Includes robot_state_publisher

**sensor.launch.py.template** - Sensor configuration launcher
- Attaches sensors to robot
- Configures sensor parameters from YAML
- Publishes sensor data to ROS 2 topics

### URDF Templates

**simple_robot.urdf.template** - Basic robot structure
- Single rigid body with proper inertia
- Visual and collision meshes
- Gazebo material properties

**mobile_robot.urdf.template** - Differential drive robot
- Base with wheels
- Joint definitions
- Gazebo differential drive plugin

**sensor_robot.urdf.template** - Robot with sensors
- Base robot structure
- LiDAR sensor mount
- Depth camera mount
- IMU integration

## Usage

1. **Copy template to your package**:
   ```bash
   cp src/textbook_templates/package.xml.template src/my_package/package.xml
   ```

2. **Replace placeholders**:
   ```bash
   sed -i 's/PACKAGE_NAME/my_package/g' src/my_package/package.xml
   ```

3. **Customize for your needs**:
   - Uncomment required dependencies
   - Add your specific configuration
   - Follow inline comments for guidance

## Template Conventions

- `PACKAGE_NAME`: Your ROS 2 package name
- `ROBOT_NAME`: Your robot's name (used in URDF and launch files)
- `SENSOR_TYPE`: Sensor type (lidar, camera, imu)
- `[OPTIONAL]`: Sections that can be removed if not needed
- `# TODO`: Sections requiring your input

## Contributing

Found an issue with a template or have an improvement? Open an issue or submit a pull request!
