class TreeFormatter:
    """Format tree structures for display."""

    def __init__(self, use_unicode=True, show_prefixes=False):
        if use_unicode:
            self.symbols = {
                "branch": "├── ",
                "last_branch": "└── ",
                "vertical": "│   ",
                "space": "    ",
            }
        else:
            self.symbols = {
                "branch": "|-- ",
                "last_branch": "`-- ",
                "vertical": "|   ",
                "space": "    ",
            }

        # Set prefixes based on show_prefixes option
        if show_prefixes:
            self.symbols.update(
                {
                    "topic": "topic: ",
                    "node": "node: ",
                    "namespace": "ns: ",
                    "service": "service: ",
                    "publisher": "pub: ",
                    "subscriber": "sub: ",
                    "server": "srv: ",
                    "client": "cli: ",
                }
            )
        else:
            self.symbols.update(
                {
                    "topic": "",
                    "node": "",
                    "namespace": "",
                    "service": "",
                    "publisher": "↑ publishes: ",
                    "subscriber": "↓ subscribes: ",
                    "server": "> serves: ",
                    "client": "< calls: ",
                }
            )

    def format_topic_tree(self, tree, show_types=False, show_connections=False, connections=None):
        """Format topic tree for display."""
        if not tree:
            return "No topics found.\n"

        lines = []
        self._format_tree_recursive(tree, "", True, lines, "topic", show_types, show_connections, connections)
        return "\n".join(lines)

    def format_node_tree(self, tree, show_connections=False, connections=None):
        """Format node tree for display."""
        if not tree:
            return "No nodes found.\n"

        lines = []
        self._format_tree_recursive(tree, "", True, lines, "node", False, show_connections, connections)
        return "\n".join(lines)

    def format_service_tree(self, tree, show_types=False, show_connections=False, connections=None):
        """Format service tree for display."""
        if not tree:
            return "No services found.\n"

        lines = []
        self._format_tree_recursive(tree, "", True, lines, "service", show_types, show_connections, connections)
        return "\n".join(lines)

    def _format_tree_recursive(
        self, tree, prefix, is_last, lines, item_type, show_types=False, show_connections=False, connections=None
    ):
        """Recursively format tree structure."""
        if not isinstance(tree, dict):
            return

        items = list(tree.items())
        for i, (key, value) in enumerate(items):
            is_last_item = i == len(items) - 1
            current_prefix = prefix + (self.symbols["last_branch"] if is_last_item else self.symbols["branch"])

            # Check if this is a leaf node (topic or node)
            if isinstance(value, dict) and "_topic_info" in value:
                # This is a topic
                topic_info = value["_topic_info"]
                symbol = self.symbols["topic"]
                display_line = f"{current_prefix}{symbol}{key}"

                if show_types and "types" in topic_info:
                    types_str = ", ".join(topic_info["types"])
                    display_line += f" ({types_str})"

                lines.append(display_line)

                # Show connections if requested
                if show_connections and connections:
                    self._add_connection_info(
                        lines, prefix, is_last_item, topic_info["full_name"], connections, "topic"
                    )

            elif isinstance(value, dict) and "_node_info" in value:
                # This is a node
                node_info = value["_node_info"]
                symbol = self.symbols["node"]
                display_line = f"{current_prefix}{symbol}{key}"
                lines.append(display_line)

                # Show connections if requested
                if show_connections and connections:
                    self._add_connection_info(lines, prefix, is_last_item, node_info["full_name"], connections, "node")

            elif isinstance(value, dict) and "_service_info" in value:
                # This is a service
                service_info = value["_service_info"]
                symbol = self.symbols["service"]
                display_line = f"{current_prefix}{symbol}{key}"

                if show_types and "type" in service_info:
                    display_line += f" ({service_info['type']})"

                lines.append(display_line)

                # Show connections if requested
                if show_connections and connections:
                    self._add_connection_info(
                        lines, prefix, is_last_item, service_info["full_name"], connections, "service"
                    )

            else:
                # This is a namespace/directory, but check if it also has a topic or service
                if isinstance(value, dict) and "_self_topic_info" in value:
                    # This namespace is also a topic
                    topic_info = value["_self_topic_info"]
                    symbol = self.symbols["topic"]
                    display_line = f"{current_prefix}{symbol}{key}"

                    if show_types and "types" in topic_info:
                        types_str = ", ".join(topic_info["types"])
                        display_line += f" ({types_str})"

                    lines.append(display_line)

                    # Show connections if requested
                    if show_connections and connections:
                        self._add_connection_info(
                            lines, prefix, is_last_item, topic_info["full_name"], connections, "topic"
                        )

                    # Also recurse into subdirectory for children
                    next_prefix = prefix + (self.symbols["space"] if is_last_item else self.symbols["vertical"])
                    # Create a copy without the _self_topic_info for recursion
                    child_value = {k: v for k, v in value.items() if k != "_self_topic_info"}
                    if child_value:  # Only recurse if there are children
                        self._format_tree_recursive(
                            child_value, next_prefix, False, lines, item_type, show_types, show_connections, connections
                        )
                elif isinstance(value, dict) and "_self_service_info" in value:
                    # This namespace is also a service
                    service_info = value["_self_service_info"]
                    symbol = self.symbols["service"]
                    display_line = f"{current_prefix}{symbol}{key}"

                    if show_types and "type" in service_info:
                        display_line += f" ({service_info['type']})"

                    lines.append(display_line)

                    # Show connections if requested
                    if show_connections and connections:
                        self._add_connection_info(
                            lines, prefix, is_last_item, service_info["full_name"], connections, "service"
                        )

                    # Also recurse into subdirectory for children
                    next_prefix = prefix + (self.symbols["space"] if is_last_item else self.symbols["vertical"])
                    # Create a copy without the _self_service_info for recursion
                    child_value = {k: v for k, v in value.items() if k != "_self_service_info"}
                    if child_value:  # Only recurse if there are children
                        self._format_tree_recursive(
                            child_value, next_prefix, False, lines, item_type, show_types, show_connections, connections
                        )
                else:
                    # Pure namespace/directory
                    symbol = self.symbols["namespace"]
                    lines.append(f"{current_prefix}{symbol}{key}")

                    # Recurse into subdirectory
                    next_prefix = prefix + (self.symbols["space"] if is_last_item else self.symbols["vertical"])
                    self._format_tree_recursive(
                        value, next_prefix, False, lines, item_type, show_types, show_connections, connections
                    )

    def _add_connection_info(self, lines, prefix, is_last_parent, name, connections, item_type):
        """Add connection information for topics or nodes."""
        if not connections:
            return

        connection_prefix = prefix + (self.symbols["space"] if is_last_parent else self.symbols["vertical"])

        if item_type == "topic":
            # Show publishers and subscribers for this topic
            publishers = connections.get("publishers", {}).get(name, [])
            subscribers = connections.get("subscribers", {}).get(name, [])

            if publishers:
                for i, pub in enumerate(publishers):
                    is_last_pub = (i == len(publishers) - 1) and not subscribers
                    branch = self.symbols["last_branch"] if is_last_pub else self.symbols["branch"]
                    lines.append(f"{connection_prefix}{branch}{self.symbols['publisher']}{pub}")

            if subscribers:
                for i, sub in enumerate(subscribers):
                    is_last_sub = i == len(subscribers) - 1
                    branch = self.symbols["last_branch"] if is_last_sub else self.symbols["branch"]
                    lines.append(f"{connection_prefix}{branch}{self.symbols['subscriber']}{sub}")

        elif item_type == "node":
            # Show published and subscribed topics for this node
            # Try both with and without leading slash for node name matching
            node_variations = [name]
            if name.startswith("/"):
                node_variations.append(name[1:])  # Remove leading slash
            else:
                node_variations.append("/" + name)  # Add leading slash

            published = []
            subscribed = []

            for node_name in node_variations:
                if node_name in connections.get("node_pubs", {}):
                    published = connections["node_pubs"][node_name]
                    break

            for node_name in node_variations:
                if node_name in connections.get("node_subs", {}):
                    subscribed = connections["node_subs"][node_name]
                    break

            # Also show service connections for nodes
            served_services = []
            client_services = []
            for node_name in node_variations:
                if node_name in connections.get("node_servers", {}):
                    served_services = connections["node_servers"][node_name]
                    break

            for node_name in node_variations:
                if node_name in connections.get("node_clients", {}):
                    client_services = connections["node_clients"][node_name]
                    break

            if published:
                for i, topic in enumerate(published):
                    is_last_pub = (
                        (i == len(published) - 1) and not subscribed and not served_services and not client_services
                    )
                    branch = self.symbols["last_branch"] if is_last_pub else self.symbols["branch"]
                    lines.append(f"{connection_prefix}{branch}{self.symbols['publisher']}{topic}")

            if subscribed:
                for i, topic in enumerate(subscribed):
                    is_last_sub = (i == len(subscribed) - 1) and not served_services and not client_services
                    branch = self.symbols["last_branch"] if is_last_sub else self.symbols["branch"]
                    lines.append(f"{connection_prefix}{branch}{self.symbols['subscriber']}{topic}")

            if served_services:
                for i, service in enumerate(served_services):
                    is_last_server = (i == len(served_services) - 1) and not client_services
                    branch = self.symbols["last_branch"] if is_last_server else self.symbols["branch"]
                    # Remove node name prefix from service name since it's redundant
                    service_name = service
                    for node_name in node_variations:
                        if service.startswith(node_name + "/"):
                            service_name = service[len(node_name + "/"):]
                            break
                    lines.append(f"{connection_prefix}{branch}{self.symbols['server']}{service_name}")

            if client_services:
                for i, service in enumerate(client_services):
                    is_last_client = i == len(client_services) - 1
                    branch = self.symbols["last_branch"] if is_last_client else self.symbols["branch"]
                    # Remove node name prefix from service name since it's redundant
                    service_name = service
                    for node_name in node_variations:
                        if service.startswith(node_name + "/"):
                            service_name = service[len(node_name + "/"):]
                            break
                    lines.append(f"{connection_prefix}{branch}{self.symbols['client']}{service_name}")

        elif item_type == "service":
            # For services tree, only show clients (servers are redundant since tree structure shows the providing node)
            clients = connections.get("clients", {}).get(name, [])

            if clients:
                for i, client in enumerate(clients):
                    is_last_client = i == len(clients) - 1
                    branch = self.symbols["last_branch"] if is_last_client else self.symbols["branch"]
                    lines.append(f"{connection_prefix}{branch}{self.symbols['client']}{client}")

    def format_combined_tree(self, topic_tree, node_tree, connections=None, show_connections=False):
        """Format combined view of topics and nodes."""
        lines = []

        lines.append("ROS2 System Tree")
        lines.append("")

        if topic_tree:
            lines.append("Topics:")
            topic_lines = self.format_topic_tree(
                topic_tree, show_types=True, show_connections=show_connections, connections=connections
            )
            # Indent topic tree
            for line in topic_lines.split("\n"):
                if line.strip():
                    lines.append(f"  {line}")
        else:
            lines.append("Topics: None")

        lines.append("")

        if node_tree:
            lines.append("Nodes:")
            node_lines = self.format_node_tree(node_tree, show_connections=show_connections, connections=connections)
            # Indent node tree
            for line in node_lines.split("\n"):
                if line.strip():
                    lines.append(f"  {line}")
        else:
            lines.append("Nodes: None")

        return "\n".join(lines)
