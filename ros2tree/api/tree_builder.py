import subprocess
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node


class TreeBuilder:
    """Build tree structure for ROS2 topics and nodes."""

    def __init__(self):
        self.node = None
        self.topics_cache = {}
        self.nodes_cache = {}
        self.last_update = 0
        self.cache_duration = 2.0  # Cache for 2 seconds

    def _ensure_node(self):
        """Ensure ROS2 node is initialized."""
        if self.node is None:
            if not rclpy.ok():
                rclpy.init()
            self.node = Node("ros2tree_introspection")

    def _should_refresh_cache(self):
        """Check if cache needs refreshing."""
        return time.time() - self.last_update > self.cache_duration

    def get_topic_tree(self, include_types=True):
        """Get topics organized in tree structure."""
        self._ensure_node()

        if "topics" not in self.topics_cache or self._should_refresh_cache():
            # First try the API method
            topic_names_and_types = self.node.get_topic_names_and_types()

            # Check if we're missing topics by comparing with ros2 topic list
            try:

                result = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    # Parse the output to get all topic names
                    subprocess_topics = {
                        line.strip()
                        for line in result.stdout.split("\n")
                        if line.strip() and not line.startswith("WARNING")
                    }
                    api_topics = {name for name, _ in topic_names_and_types}

                    missing_topics = subprocess_topics - api_topics
                    if missing_topics:
                        # Add missing topics with unknown types
                        for topic_name in missing_topics:
                            topic_names_and_types.append((topic_name, ["unknown"]))
            except (subprocess.SubprocessError, OSError, ValueError):
                pass

            self.topics_cache["topics"] = self._build_topic_tree(topic_names_and_types, include_types)
            self.last_update = time.time()

        return self.topics_cache["topics"]

    def get_node_tree(self):
        """Get nodes organized by namespace."""
        self._ensure_node()

        if "nodes" not in self.nodes_cache or self._should_refresh_cache():
            # Try to get full node names with namespaces using subprocess
            try:

                result = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    # Parse the output to get full node names
                    node_names = [
                        line.strip()
                        for line in result.stdout.split("\n")
                        if line.strip() and not line.startswith("WARNING")
                    ]
                else:
                    # Fallback to API method
                    node_names = self.node.get_node_names()
            except (subprocess.SubprocessError, OSError, ValueError):
                # Fallback to API method if subprocess fails
                node_names = self.node.get_node_names()

            self.nodes_cache["nodes"] = self._build_node_tree(node_names)
            self.last_update = time.time()

        return self.nodes_cache["nodes"]

    def get_node_topic_connections(self):
        """Get connections between nodes and topics."""
        self._ensure_node()

        cache_key = "connections"
        if cache_key not in self.nodes_cache or self._should_refresh_cache():
            connections = self._get_node_topic_connections()
            self.nodes_cache[cache_key] = connections
            self.last_update = time.time()

        return self.nodes_cache[cache_key]

    def _build_topic_tree(self, topic_names_and_types, include_types=True):
        """Build hierarchical topic tree from flat topic list."""
        tree = {}

        for topic_name, topic_types in topic_names_and_types:
            # Split topic path into components
            parts = [part for part in topic_name.split("/") if part]

            # Navigate/create tree structure
            current = tree
            for i, part in enumerate(parts[:-1]):  # All but last component
                if part not in current:
                    current[part] = {}
                elif "_topic_info" in current[part]:
                    # This part was already a topic, but now we need it as a namespace
                    # Convert it to a namespace that also contains the topic info
                    topic_info = current[part]["_topic_info"]
                    current[part] = {"_self_topic_info": topic_info}
                current = current[part]

            # Add final topic with its types
            final_part = parts[-1] if parts else topic_name
            topic_info = {"full_name": topic_name, "types": topic_types} if include_types else {"full_name": topic_name}

            if final_part in current and "_topic_info" not in current[final_part]:
                # This final part already exists as a namespace, add topic info to it
                current[final_part]["_topic_info"] = topic_info
            else:
                # Normal case - create new topic entry
                current[final_part] = {"_topic_info": topic_info}

        return tree

    def _build_node_tree(self, node_names):
        """Build hierarchical node tree from flat node list."""
        tree = {}

        for node_name in node_names:
            # Split node path into namespace components
            if node_name.startswith("/"):
                node_name = node_name[1:]  # Remove leading slash

            parts = [part for part in node_name.split("/") if part]

            if not parts:
                continue

            # Navigate/create tree structure
            current = tree
            for part in parts[:-1]:  # All but last component (namespace)
                if part not in current:
                    current[part] = {}
                current = current[part]

            # Add final node name
            final_part = parts[-1] if len(parts) > 1 else parts[0]
            current[final_part] = {"_node_info": {"full_name": "/" + "/".join(parts)}}

        return tree

    def _get_node_topic_connections(self):
        """Get publisher and subscriber information for nodes and topics."""
        connections = {
            "publishers": defaultdict(list),  # topic -> [nodes]
            "subscribers": defaultdict(list),  # topic -> [nodes]
            "node_pubs": defaultdict(list),  # node -> [topics]
            "node_subs": defaultdict(list),  # node -> [topics]
        }

        try:
            import re

            # Get all nodes first
            try:
                result = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    node_names = [
                        line.strip()
                        for line in result.stdout.split("\n")
                        if line.strip() and not line.startswith("WARNING")
                    ]
                else:
                    node_names = self.node.get_node_names()
            except (subprocess.SubprocessError, OSError, ValueError):
                node_names = self.node.get_node_names()

            # For each node, get its publisher and subscriber info
            for node_name in node_names:
                try:
                    result = subprocess.run(
                        ["ros2", "node", "info", node_name], capture_output=True, text=True, timeout=5
                    )
                    if result.returncode != 0:
                        continue

                    lines = result.stdout.split("\n")
                    current_section = None

                    for line in lines:
                        line = line.strip()

                        if line == "Subscribers:":
                            current_section = "subscribers"
                            continue
                        elif line == "Publishers:":
                            current_section = "publishers"
                            continue
                        elif line in ["Service Servers:", "Service Clients:", "Action Servers:", "Action Clients:"]:
                            current_section = None
                            continue

                        # Parse topic lines in format: "    /topic_name: msg_type"
                        if current_section and ":" in line and line.strip().startswith("/"):
                            # Extract topic name from indented lines
                            topic_match = re.match(r"\s*(/\S+):", line)
                            if topic_match:
                                topic_name = topic_match.group(1)

                                if current_section == "publishers":
                                    connections["publishers"][topic_name].append(node_name)
                                    connections["node_pubs"][node_name].append(topic_name)
                                elif current_section == "subscribers":
                                    connections["subscribers"][topic_name].append(node_name)
                                    connections["node_subs"][node_name].append(topic_name)

                except (subprocess.SubprocessError, OSError, ValueError, RuntimeError):
                    # Skip this node if we can't get info
                    continue

        except (subprocess.SubprocessError, OSError, ValueError, RuntimeError):
            # If all fails, return empty connections but don't crash
            pass

        return dict(connections)

    def get_service_tree(self, include_types=True):
        """Get services organized in tree structure."""
        self._ensure_node()

        cache_key = f"services_{include_types}"
        if cache_key not in self.topics_cache or self._should_refresh_cache():
            try:
                if include_types:
                    # Get services with types using ros2 service list -t
                    result = subprocess.run(
                        ["ros2", "service", "list", "-t"], capture_output=True, text=True, timeout=5
                    )
                    if result.returncode == 0:
                        service_names_and_types = []
                        for line in result.stdout.split("\n"):
                            line = line.strip()
                            if line and not line.startswith("WARNING"):
                                # Parse format: "/service_name [service_type]"
                                if " [" in line and line.endswith("]"):
                                    service_name = line.split(" [")[0]
                                    service_type = line.split(" [")[1][:-1]  # Remove closing ]
                                    service_names_and_types.append((service_name, service_type))
                                else:
                                    # Fallback if format is unexpected
                                    service_names_and_types.append((line, "unknown"))
                    else:
                        service_names_and_types = []
                else:
                    # Just get service names
                    result = subprocess.run(["ros2", "service", "list"], capture_output=True, text=True, timeout=5)
                    if result.returncode == 0:
                        service_names = [
                            line.strip()
                            for line in result.stdout.split("\n")
                            if line.strip() and not line.startswith("WARNING")
                        ]
                        service_names_and_types = [(name, None) for name in service_names]
                    else:
                        service_names_and_types = []
            except (subprocess.SubprocessError, OSError, ValueError):
                service_names_and_types = []

            self.topics_cache[cache_key] = self._build_service_tree(service_names_and_types, include_types)
            self.last_update = time.time()

        return self.topics_cache[cache_key]

    def get_service_connections(self):
        """Get connections between services and nodes."""
        self._ensure_node()

        cache_key = "service_connections"
        if cache_key not in self.nodes_cache or self._should_refresh_cache():
            connections = self._get_service_connections()
            self.nodes_cache[cache_key] = connections
            self.last_update = time.time()

        return self.nodes_cache[cache_key]

    def _build_service_tree(self, service_names_and_types, include_types=True):
        """Build hierarchical service tree from flat service list."""
        tree = {}

        for service_name, service_type in service_names_and_types:
            # Split service path into components
            parts = [part for part in service_name.split("/") if part]

            # Navigate/create tree structure
            current = tree
            for i, part in enumerate(parts[:-1]):  # All but last component
                if part not in current:
                    current[part] = {}
                elif "_service_info" in current[part]:
                    # This part was already a service, but now we need it as a namespace
                    # Convert it to a namespace that also contains the service info
                    service_info = current[part]["_service_info"]
                    current[part] = {"_self_service_info": service_info}
                current = current[part]

            # Add final service
            final_part = parts[-1] if parts else service_name
            service_info = {"full_name": service_name}
            if include_types and service_type:
                service_info["type"] = service_type

            if final_part in current and "_service_info" not in current[final_part]:
                # This final part already exists as a namespace, add service info to it
                current[final_part]["_service_info"] = service_info
            else:
                # Normal case - create new service entry
                current[final_part] = {"_service_info": service_info}

        return tree

    def _get_service_connections(self):
        """Get service server and client information."""
        connections = {
            "servers": defaultdict(list),  # service -> [nodes]
            "clients": defaultdict(list),  # service -> [nodes]
            "node_servers": defaultdict(list),  # node -> [services]
            "node_clients": defaultdict(list),  # node -> [services]
        }

        try:
            import re

            # Get all nodes first
            try:
                result = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    node_names = [
                        line.strip()
                        for line in result.stdout.split("\n")
                        if line.strip() and not line.startswith("WARNING")
                    ]
                else:
                    node_names = self.node.get_node_names()
            except (subprocess.SubprocessError, OSError, ValueError):
                node_names = self.node.get_node_names()

            # For each node, get its service server and client info
            for node_name in node_names:
                try:
                    result = subprocess.run(
                        ["ros2", "node", "info", node_name], capture_output=True, text=True, timeout=5
                    )
                    if result.returncode != 0:
                        continue

                    lines = result.stdout.split("\n")
                    current_section = None

                    for line in lines:
                        line = line.strip()

                        if line == "Service Servers:":
                            current_section = "servers"
                            continue
                        elif line == "Service Clients:":
                            current_section = "clients"
                            continue
                        elif line in ["Subscribers:", "Publishers:", "Action Servers:", "Action Clients:"]:
                            current_section = None
                            continue

                        # Parse service lines in format: "    /service_name: service_type"
                        if current_section and ":" in line and line.strip().startswith("/"):
                            # Extract service name from indented lines
                            service_match = re.match(r"\s*(/\S+):", line)
                            if service_match:
                                service_name = service_match.group(1)

                                if current_section == "servers":
                                    connections["servers"][service_name].append(node_name)
                                    connections["node_servers"][node_name].append(service_name)
                                elif current_section == "clients":
                                    connections["clients"][service_name].append(node_name)
                                    connections["node_clients"][node_name].append(service_name)

                except (subprocess.SubprocessError, OSError, ValueError, RuntimeError):
                    # Skip this node if we can't get info
                    continue

        except (subprocess.SubprocessError, OSError, ValueError, RuntimeError):
            # If all fails, return empty connections but don't crash
            pass

        return dict(connections)

    def cleanup(self):
        """Clean up resources."""
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
