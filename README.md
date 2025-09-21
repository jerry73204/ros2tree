# ros2tree

A ROS2 command extension that displays topics and nodes in a tree view format.

## Overview

`ros2tree` provides an intuitive way to visualize the structure of your ROS2 system by displaying topics and nodes in a hierarchical tree format. This makes it easier to understand the organization and relationships within your ROS2 workspace.

## Features

- 🌳 **Tree View Display**: Hierarchical visualization of topics and nodes
- 📡 **Topic Inspection**: View topics organized by namespace with type information
- 🔧 **Node Inspection**: View nodes organized by namespace
- 🔗 **Connection Analysis**: Show publisher/subscriber relationships
- 🎨 **Unicode/ASCII Support**: Choose between Unicode symbols or ASCII characters
- ⚡ **Fast Caching**: Efficient caching for improved performance

## Installation

### From Source (Recommended for Development)

```bash
# Clone and build with colcon
cd ~/ros2_ws/src
git clone <your-repo-url> ros2tree
cd ~/ros2_ws
colcon build --packages-select ros2tree
source install/setup.bash
```

### Alternative: Direct pip install

```bash
cd ros2tree
pip install .
```

## Usage

### Basic Commands

```bash
# View all topics in tree format
ros2 tree topics

# View all nodes in tree format
ros2 tree nodes

# View both topics and nodes together
ros2 tree all
```

### Advanced Options

```bash
# Show topic types
ros2 tree topics

# Hide topic types
ros2 tree topics --no-types

# Show connections (publishers/subscribers)
ros2 tree topics --connections
ros2 tree nodes --connections

# Use ASCII characters instead of Unicode
ros2 tree all --no-unicode

# Combined view without connections
ros2 tree all --no-connections
```

## Example Output

### Topics Tree
```
📡 Topics:
  ├── 📁 parameter_events/
  │   └── 📡 parameter_events (rcl_interfaces/msg/ParameterEvent)
  ├── 📁 rosout/
  │   └── 📡 rosout (rcl_interfaces/msg/Log)
  └── 📁 turtle1/
      ├── 📡 cmd_vel (geometry_msgs/msg/Twist)
      └── 📡 pose (turtlesim/msg/Pose)
```

### Nodes Tree
```
🔧 Nodes:
  ├── 🔧 ros2tree_introspection
  ├── 🔧 teleop_turtle
  └── 🔧 turtlesim
```

### Combined View with Connections
```
🌳 ROS2 System Tree

📡 Topics:
  ├── 📡 /turtle1/cmd_vel (geometry_msgs/msg/Twist)
  │   ├── ➡️  /teleop_turtle
  │   └── ⬅️  /turtlesim
  └── 📡 /turtle1/pose (turtlesim/msg/Pose)
      └── ➡️  /turtlesim

🔧 Nodes:
  ├── 🔧 teleop_turtle
  │   └── ➡️  /turtle1/cmd_vel
  └── 🔧 turtlesim
      ├── ➡️  /turtle1/pose
      └── ⬅️  /turtle1/cmd_vel
```

## Requirements

- ROS2 (Humble or later)
- Python 3.8+
- rclpy

## Development

### Testing

```bash
# Run basic functionality test with demo nodes
ros2 run demo_nodes_cpp talker &
ros2 tree topics
ros2 tree nodes
ros2 tree all
```

### Code Style

This project follows standard Python conventions:
- Line length: 120 characters
- Formatter: black (if available)
- Linter: flake8

## License

Apache License 2.0

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.