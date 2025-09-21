from setuptools import find_packages, setup
import os

package_name = "ros2tree"

# Determine if we're in a ROS2 build environment
in_ros2_build = os.environ.get('COLCON', False) or os.environ.get('AMENT_PREFIX_PATH', False)

# Common setup parameters
common_params = {
    "name": package_name,
    "version": "0.1.0",
    "packages": find_packages(exclude=["test", "tests"]),
    "zip_safe": True,
    "maintainer": "ROS2 Tree Developer",
    "maintainer_email": "developer@example.com",
    "description": "ROS2 command extension for displaying topics and nodes in tree view",
    "license": "Apache-2.0",
    "entry_points": {
        "ros2cli.command": [
            "tree = ros2tree.command.tree:TreeCommand",
        ],
        "ros2cli.extension_point": [
            "ros2tree.verb = ros2tree.verb:VerbExtension",
        ],
        "ros2tree.verb": [
            "topics = ros2tree.verb.topics:TopicsVerb",
            "nodes = ros2tree.verb.nodes:NodesVerb",
            "all = ros2tree.verb.all:AllVerb",
        ],
    },
}

# ROS2/Colcon specific configuration
if in_ros2_build:
    common_params.update({
        "data_files": [
            ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
            ("share/" + package_name, ["package.xml"]),
        ],
        "install_requires": [
            "ros2cli",
            "setuptools",
        ],
        "extras_require": {
            "dev": [
                "pytest",
                "pytest-cov",
            ],
        },
    })
else:
    # Pure pip installation
    common_params.update({
        "install_requires": [
            "setuptools",
            # ros2cli is assumed to be provided by ROS2 environment
        ],
        "python_requires": ">=3.8",
        "long_description": open("README.md").read() if os.path.exists("README.md") else "",
        "long_description_content_type": "text/markdown",
        "classifiers": [
            "Development Status :: 3 - Alpha",
            "Intended Audience :: Developers",
            "License :: OSI Approved :: Apache Software License",
            "Operating System :: POSIX :: Linux",
            "Programming Language :: Python :: 3",
            "Programming Language :: Python :: 3.8",
            "Programming Language :: Python :: 3.9",
            "Programming Language :: Python :: 3.10",
            "Programming Language :: Python :: 3.11",
        ],
    })

setup(**common_params)