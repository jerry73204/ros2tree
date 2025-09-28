# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ros2tree is a ROS2 command extension that displays topics, nodes, and services in a hierarchical tree format. It integrates with the ros2cli system to provide intuitive visualization of ROS2 systems.

Repository: https://github.com/jerry73204/ros2tree

## Build & Development Commands

### Building the Project
```bash
# ROS2 colcon build (recommended for development)
cd ~/ros2_ws/src
git clone https://github.com/jerry73204/ros2tree.git
cd ~/ros2_ws
colcon build --packages-select ros2tree
source install/setup.bash

# Alternative: Direct pip install
cd ros2tree
pip install .

# Build wheel for distribution
make wheel
```

### Testing Commands
```bash
# Basic functionality test with demo nodes
ros2 run demo_nodes_cpp talker &
ros2 tree topics
ros2 tree nodes
ros2 tree services
ros2 tree all

# Test connection analysis
ros2 tree topics --connections
ros2 tree nodes --connections
ros2 tree services --connections
ros2 tree all --verbose

# Test service clients (requires separate terminals)
ros2 run demo_nodes_py add_two_ints_server &
ros2 run demo_nodes_py add_two_ints_client
ros2 tree services --connections

# Test display modes
ros2 tree all --no-unicode
ros2 tree topics --show-prefixes
ros2 tree topics --no-types
```

### Code Quality
- Line length: 120 characters
- Formatter: black (if available)
- Linter: flake8
- ROS2 linting tools: ament_flake8, ament_pep257, ament_copyright

## Architecture

### Core Components

1. **Command Extension System**: Uses ros2cli plugin architecture
   - `ros2tree/command/tree.py`: Main command entry point
   - `ros2tree/verb/__init__.py`: Verb extension point registration

2. **Tree Builder (`ros2tree/api/tree_builder.py`)**: Core data collection and caching
   - **TreeBuilder class**: Main API for collecting ROS2 system information
   - Implements intelligent caching (2-second cache duration)
   - Uses hybrid approach: rclpy APIs + subprocess calls for completeness
   - Methods: `get_topic_tree()`, `get_node_tree()`, `get_service_tree()`, `get_*_connections()`

3. **Tree Formatter (`ros2tree/api/tree_formatter.py`)**: Display formatting
   - **TreeFormatter class**: Handles Unicode/ASCII tree rendering
   - Supports connection visualization (publishers/subscribers/services)
   - Configurable prefix display for grep-friendly output

4. **Verb Implementations**: Individual command handlers
   - `topics.py`: Topic tree display
   - `nodes.py`: Node tree display
   - `services.py`: Service tree display
   - `all.py`: Combined view with cross-references

### Data Flow Architecture

1. **Data Collection**: TreeBuilder uses subprocess + rclpy for robustness
   - Primary: rclpy node introspection APIs
   - Fallback: `ros2 topic list`, `ros2 node list`, `ros2 service list` commands
   - Connection analysis: `ros2 node info` parsing

2. **Tree Building**: Hierarchical namespace organization
   - Topics/services organized by `/namespace/hierarchy`
   - Nodes organized by namespace structure
   - Special handling for namespace conflicts (when path is both namespace and endpoint)

3. **Connection Analysis**: Cross-reference relationships
   - Publisher/subscriber mappings for topics
   - Service server/client relationships
   - Node-centric view of connections

### Plugin Architecture

ros2tree follows ros2cli extension patterns:
- **Command Extension**: `ros2tree.command.tree:TreeCommand`
- **Verb Extension Point**: `ros2tree.verb:VerbExtension`
- **Individual Verbs**: Registered in setup.py entry_points

### Caching Strategy

TreeBuilder implements intelligent caching to avoid expensive ROS2 introspection calls:
- 2-second cache duration for all data types
- Separate cache keys for different data views (topics with/without types, etc.)
- Cache invalidation based on timestamps

### Error Handling

- Graceful fallbacks when rclpy APIs fail (uses subprocess)
- Timeout protection on subprocess calls (5-second limit)
- Exception isolation prevents single node failures from crashing entire tree

## Development Notes

### Adding New Verb Commands

1. Create new file in `ros2tree/verb/`
2. Implement `VerbExtension` subclass with `add_arguments()` and `main()` methods
3. Register in `setup.py` under `ros2tree.verb` entry points
4. Follow existing patterns for TreeBuilder/TreeFormatter usage

### Extending Tree Data

The tree data structures use special keys for metadata:
- `_topic_info`: Topic endpoint data
- `_node_info`: Node endpoint data
- `_service_info`: Service endpoint data
- `_self_*_info`: When namespace path is also an endpoint

### Connection Data Format

Connection information follows this structure:
```python
{
    "publishers": {"/topic": ["node1", "node2"]},
    "subscribers": {"/topic": ["node3"]},
    "node_pubs": {"node1": ["/topic1", "/topic2"]},
    "node_subs": {"node1": ["/topic3"]},
    "servers": {"/service": ["server_node"]},
    "clients": {"/service": ["client_node1", "client_node2"]},
    "node_servers": {"server_node": ["/service1", "/service2"]},
    "node_clients": {"client_node": ["/service1"]}
}
```

## Important Implementation Notes

### Service Client Detection
- **Already implemented**: Service clients are fully detected and displayed
- Uses both `ros2 node info` subprocess parsing and rclpy `get_client_names_and_types_by_node()` API
- Displays in tree with "< calls:" prefix for service clients
- Example: `├── add_two_ints │ └── < calls: /service_client_node`

### File Structure
- **README.md**: User-facing documentation (no development content)
- **CONTRIBUTING.md**: All development and contribution guidelines
- **LICENSE**: Apache License 2.0 with Lin Hsiang-Jui as copyright holder
- **RELEASE_NOTES_v{version}.md**: Release notes for current version
- **CLAUDE.md**: This file for Claude Code guidance

### Distribution
- Wheel filename: `ros2tree-{version}-py3-none-any.whl`
- Built with: `make wheel`
- Located in: `dist/` directory after build