from ros2cli.verb import VerbExtension

from ros2tree.api.tree_builder import TreeBuilder
from ros2tree.api.tree_formatter import TreeFormatter


class TopicsVerb(VerbExtension):
    """Display ROS2 topics in tree view."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("--no-types", action="store_true", help="Do not display topic types")
        parser.add_argument(
            "--no-unicode", action="store_true", help="Use ASCII characters instead of Unicode for tree display"
        )
        parser.add_argument("--connections", action="store_true", help="Show publisher and subscriber connections")
        parser.add_argument(
            "--show-prefixes", action="store_true", help="Show type prefixes (topic:, ns:) for better grep filtering"
        )
        parser.add_argument(
            "--verbose",
            "-v",
            action="store_true",
            help="Show detailed information: publishers and subscribers for each topic",
        )

    def main(self, *, args, parser):
        builder = TreeBuilder()
        formatter = TreeFormatter(use_unicode=not args.no_unicode, show_prefixes=args.show_prefixes)

        try:
            # Get topic tree
            topic_tree = builder.get_topic_tree(include_types=not args.no_types)

            # Get connections if requested or verbose enabled
            connections = None
            if args.connections or args.verbose:
                connections = builder.get_node_topic_connections()

            # Format and display
            output = formatter.format_topic_tree(
                topic_tree,
                show_types=not args.no_types,
                show_connections=args.connections or args.verbose,
                connections=connections,
            )
            print(output)

        except (RuntimeError, ValueError, OSError) as e:
            print(f"Error retrieving topics: {e}")
            return 1
        finally:
            builder.cleanup()

        return 0
