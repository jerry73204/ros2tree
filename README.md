# ros2tree

A ROS2 command-line extension that displays topics, nodes, and services in a hierarchical tree format.

## Overview

ros2tree provides an intuitive way to visualize the structure of your ROS2 system. It organizes topics, nodes, and services by their namespace hierarchy, making it easier to understand complex ROS2 systems at a glance. The tool integrates seamlessly with the ros2 CLI, providing commands like `ros2 tree topics`, `ros2 tree nodes`, and `ros2 tree services`.

## Installation

### From PyPI (Recommended)

```bash
pip install ros2tree
```

### Alternative: ROS2 Workspace Installation

If you're working within a ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/jerry73204/ros2tree.git
cd ~/ros2_ws
colcon build --packages-select ros2tree
source install/setup.bash
```

## Usage

### Basic Commands

Display topics in a tree format:
```bash
ros2 tree topics
```

Display nodes in a tree format:
```bash
ros2 tree nodes
```

Display services in a tree format:
```bash
ros2 tree services
```

Display both topics and nodes together:
```bash
ros2 tree all
```

### Example Output

#### Topics Tree
```
├── calibration
│   ├── aruco_locator
│   │   ├── aruco_detections (vision_msgs/msg/Detection2DArray)
│   │   └── image_with_detections (sensor_msgs/msg/Image)
│   └── extrinsic_solver
│       ├── calibration_quality (std_msgs/msg/String)
│       └── extrinsic_transform (geometry_msgs/msg/TransformStamped)
├── chatter (std_msgs/msg/String)
├── parameter_events (rcl_interfaces/msg/ParameterEvent)
└── rosout (rcl_interfaces/msg/Log)
```

#### Nodes Tree
```
├── calibration
│   ├── aruco_locator
│   │   └── aruco_locator
│   ├── calibration_board_locator
│   │   └── calibration_board_locator
│   └── extrinsic_solver
│       └── extrinsic_solver_node
├── ros2tree_introspection
└── talker
```

#### Services Tree
```
├── add_two_ints
├── calibration
│   ├── aruco_locator
│   │   └── aruco_locator
│   │       ├── describe_parameters
│   │       ├── get_parameters
│   │       ├── list_parameters
│   │       └── set_parameters
│   └── extrinsic_solver
│       └── extrinsic_solver_node
│           ├── describe_parameters
│           ├── get_parameters
│           └── set_parameters
└── talker
    ├── describe_parameters
    └── set_parameters
```

## Advanced Usage

### Connection Analysis

Show publisher and subscriber connections for topics:
```bash
ros2 tree topics --connections
```

Show connections for nodes (publishers, subscribers, service servers/clients):
```bash
ros2 tree nodes --connections
```

Show service server and client connections:
```bash
ros2 tree services --connections
```

Example with connections:
```
├── add_two_ints
│   └── < calls: /service_client_node
├── chatter (std_msgs/msg/String)
│   ├── ↑ publishes: /talker
│   └── ↓ subscribes: /listener
```

### Display Options

Hide topic types:
```bash
ros2 tree topics --no-types
```

Use ASCII characters instead of Unicode (for better compatibility):
```bash
ros2 tree all --no-unicode
```

Show type prefixes for easier grep filtering:
```bash
ros2 tree topics --show-prefixes
```

Verbose output with detailed connection information:
```bash
ros2 tree all --verbose
```

Combined view without connections:
```bash
ros2 tree all --no-connections
```

### Command Options Summary

#### `ros2 tree topics`
- `--no-types`: Do not display topic types
- `--no-unicode`: Use ASCII characters instead of Unicode
- `--connections`: Show publisher and subscriber connections
- `--show-prefixes`: Show type prefixes (topic:, ns:) for grep filtering
- `--verbose, -v`: Show detailed connection information

#### `ros2 tree nodes`
- `--no-unicode`: Use ASCII characters instead of Unicode
- `--connections`: Show node connections (publishers, subscribers, services)
- `--show-prefixes`: Show type prefixes (node:, ns:) for grep filtering
- `--verbose, -v`: Show detailed connection information

#### `ros2 tree services`
- `--no-unicode`: Use ASCII characters instead of Unicode
- `--connections`: Show service server and client connections
- `--show-prefixes`: Show type prefixes (service:, ns:) for grep filtering
- `--verbose, -v`: Show detailed connection information

#### `ros2 tree all`
- `--no-types`: Do not display topic types
- `--no-unicode`: Use ASCII characters instead of Unicode
- `--no-connections`: Do not show connections between nodes and topics
- `--show-prefixes`: Show type prefixes for grep filtering
- `--verbose, -v`: Show detailed connection information

## Requirements

- ROS2 (Humble or later recommended)
- Python 3.8 or higher
- rclpy (provided by ROS2 installation)

## Development

For development setup, testing procedures, code style guidelines, and contribution instructions, see [CONTRIBUTING.md](CONTRIBUTING.md).

## License

Copyright 2025 Lin Hsiang-Jui

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

See the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests on the [GitHub repository](https://github.com/jerry73204/ros2tree).
