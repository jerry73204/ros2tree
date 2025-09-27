from ros2cli.verb import VerbExtension
from ros2tree.api.tree_builder import TreeBuilder
from ros2tree.api.tree_formatter import TreeFormatter


class AllVerb(VerbExtension):
    """Display both ROS2 topics and nodes in tree view."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("--no-types", action="store_true", help="Do not display topic types")
        parser.add_argument(
            "--no-unicode", action="store_true", help="Use ASCII characters instead of Unicode for tree display"
        )
        parser.add_argument(
            "--no-connections", action="store_true", help="Do not show connections between nodes and topics"
        )
        parser.add_argument(
            "--show-prefixes",
            action="store_true",
            help="Show type prefixes (topic:, node:, ns:) for better grep filtering",
        )
        parser.add_argument(
            "--verbose",
            "-v",
            action="store_true",
            help="Show detailed information: connections between topics and nodes",
        )

    def main(self, *, args, parser):
        builder = TreeBuilder()
        formatter = TreeFormatter(use_unicode=not args.no_unicode, show_prefixes=args.show_prefixes)

        try:
            # Get both trees
            topic_tree = builder.get_topic_tree(include_types=not args.no_types)
            node_tree = builder.get_node_tree()

            # Get connections only if verbose enabled and not explicitly disabled
            connections = None
            if args.verbose and not args.no_connections:
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

            # Format and display combined view
            output = formatter.format_combined_tree(
                topic_tree, node_tree, connections, show_connections=args.verbose and not args.no_connections
            )
            print(output)

        except Exception as e:
            print(f"Error retrieving ROS2 system information: {e}")
            return 1
        finally:
            builder.cleanup()

        return 0
