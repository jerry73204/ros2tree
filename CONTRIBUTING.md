# Contributing to ros2tree

Thank you for your interest in contributing to ros2tree! This document provides guidelines for developers who want to contribute to the project.

## Development Setup

### Setting up the Development Environment

1. Clone the repository:
   ```bash
   git clone https://github.com/jerry73204/ros2tree.git
   cd ros2tree
   ```

2. Install in development mode:
   ```bash
   # Option 1: ROS2 workspace installation (recommended for ROS2 development)
   cd ~/ros2_ws/src
   git clone https://github.com/jerry73204/ros2tree.git
   cd ~/ros2_ws
   colcon build --packages-select ros2tree
   source install/setup.bash

   # Option 2: Direct pip installation
   cd ros2tree
   pip install -e .
   ```

## Testing

### Basic Functionality Testing

Run basic functionality tests with demo nodes:

```bash
# Start a demo talker
ros2 run demo_nodes_cpp talker &

# Test the commands
ros2 tree topics
ros2 tree nodes
ros2 tree services
ros2 tree all

# Test with connections
ros2 tree topics --connections
ros2 tree nodes --connections
ros2 tree services --connections
ros2 tree all --verbose

# Clean up
pkill -f talker
```

### Testing with Service Clients

To test service client detection:

```bash
# Terminal 1: Start service server
ros2 run demo_nodes_py add_two_ints_server

# Terminal 2: Start persistent client (or run multiple times)
ros2 run demo_nodes_py add_two_ints_client

# Terminal 3: Test service detection
ros2 tree services --connections
```

### Testing Different Display Modes

```bash
# Test ASCII mode
ros2 tree all --no-unicode

# Test with prefixes
ros2 tree topics --show-prefixes

# Test without types
ros2 tree topics --no-types
```

## Code Style

This project follows standard Python conventions:

- **Line length**: 120 characters
- **Formatter**: black (if available)
- **Linter**: flake8, ament_flake8, ament_pep257
- **Import style**: Standard library imports first, then third-party, then local imports
- **Docstrings**: Use triple quotes for class and function documentation

### Running Code Quality Checks

If you have the linting tools installed:

```bash
# Run flake8
flake8 ros2tree/

# Run black (formatting)
black ros2tree/

# ROS2-specific linting (if in ROS2 workspace)
ament_flake8 ros2tree/
ament_pep257 ros2tree/
```

## Building and Distribution

### Building the Wheel

To build a distributable wheel package:

```bash
make wheel
```

The wheel file will be created in the `dist/` directory as `ros2tree-{version}-py3-none-any.whl`.

### Building in ROS2 Workspace

```bash
cd ~/ros2_ws
colcon build --packages-select ros2tree
```

## Architecture Overview

### Core Components

- **TreeBuilder** (`ros2tree/api/tree_builder.py`): Data collection and caching
- **TreeFormatter** (`ros2tree/api/tree_formatter.py`): Display formatting and tree rendering
- **Command Extensions** (`ros2tree/command/`): ros2cli integration
- **Verb Implementations** (`ros2tree/verb/`): Individual command handlers

### Adding New Features

#### Adding a New Verb Command

1. Create a new file in `ros2tree/verb/`
2. Implement `VerbExtension` subclass with `add_arguments()` and `main()` methods
3. Register in `setup.py` under `ros2tree.verb` entry points
4. Follow existing patterns for TreeBuilder/TreeFormatter usage

Example:
```python
from ros2cli.verb import VerbExtension
from ros2tree.api.tree_builder import TreeBuilder
from ros2tree.api.tree_formatter import TreeFormatter

class MyNewVerb(VerbExtension):
    """My new command description."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("--my-option", help="My option help")

    def main(self, *, args, parser):
        builder = TreeBuilder()
        formatter = TreeFormatter()
        try:
            # Your implementation here
            pass
        finally:
            builder.cleanup()
        return 0
```

#### Extending Tree Data

The tree data structures use special keys for metadata:
- `_topic_info`: Topic endpoint data
- `_node_info`: Node endpoint data
- `_service_info`: Service endpoint data
- `_self_*_info`: When namespace path is also an endpoint

## Contribution Guidelines

### Pull Request Process

1. Fork the repository
2. Create a feature branch from main
3. Make your changes following the code style guidelines
4. Test your changes thoroughly
5. Update documentation if needed
6. Submit a pull request with a clear description

### Commit Message Format

Use clear, descriptive commit messages:
- Use the imperative mood ("Add feature" not "Added feature")
- Keep the first line under 50 characters
- Reference issues and pull requests when applicable

Examples:
```
Add support for action introspection
Fix Unicode display issues on Windows
Update documentation for new connection options
```

### Issue Reporting

When reporting issues:
- Use a clear, descriptive title
- Provide ROS2 version and distribution
- Include sample output demonstrating the issue
- Provide steps to reproduce

## Development Notes

### Caching Strategy

TreeBuilder implements intelligent caching (2-second duration) to avoid expensive ROS2 introspection calls. When modifying caching logic:
- Maintain cache invalidation based on timestamps
- Use separate cache keys for different data views
- Ensure thread safety if adding concurrent features

### Error Handling

- Use graceful fallbacks when rclpy APIs fail (subprocess approach)
- Implement timeout protection on subprocess calls (5-second limit)
- Isolate exceptions to prevent single node failures from crashing the entire tree

### Connection Data Format

Connection information follows this structure:
```python
{
    "publishers": {"/topic": ["node1", "node2"]},
    "subscribers": {"/topic": ["node3"]},
    "node_pubs": {"node1": ["/topic1", "/topic2"]},
    "node_subs": {"node1": ["/topic3"]},
    # Similar patterns for services
}
```

## Getting Help

- Check existing issues and discussions on GitHub
- Review the codebase documentation in `CLAUDE.md`
- Look at existing verb implementations for patterns
- Test with the demo nodes provided by ROS2

## License

By contributing to this project, you agree that your contributions will be licensed under the Apache License 2.0.