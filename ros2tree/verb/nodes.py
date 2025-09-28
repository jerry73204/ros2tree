from ros2cli.verb import VerbExtension

from ros2tree.api.tree_builder import TreeBuilder
from ros2tree.api.tree_formatter import TreeFormatter


class NodesVerb(VerbExtension):
    """Display ROS2 nodes in tree view."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "--no-unicode", action="store_true", help="Use ASCII characters instead of Unicode for tree display"
        )
        parser.add_argument(
            "--connections", action="store_true", help="Show published and subscribed topics for each node"
        )
        parser.add_argument(
            "--show-prefixes", action="store_true", help="Show type prefixes (node:) for better grep filtering"
        )
        parser.add_argument(
            "--verbose",
            "-v",
            action="store_true",
            help="Show detailed information: published and subscribed topics for each node",
        )

    def main(self, *, args, parser):
        builder = TreeBuilder()
        formatter = TreeFormatter(use_unicode=not args.no_unicode, show_prefixes=args.show_prefixes)

        try:
            # Get node tree
            node_tree = builder.get_node_tree()

            # Get connections if requested or verbose enabled
            connections = None
            if args.connections or args.verbose:
                connections = builder.get_node_topic_connections()
                # Also get service connections in verbose mode
                service_connections = builder.get_service_connections()
                # Merge service connections into the main connections dict
                if service_connections:
                    connections.update(
                        {
                            "node_servers": service_connections.get("node_servers", {}),
                            "node_clients": service_connections.get("node_clients", {}),
                        }
                    )

            # Format and display
            output = formatter.format_node_tree(
                node_tree, show_connections=args.connections or args.verbose, connections=connections
            )
            print(output)

        except (RuntimeError, ValueError, OSError) as e:
            print(f"Error retrieving nodes: {e}")
            return 1
        finally:
            builder.cleanup()

        return 0
