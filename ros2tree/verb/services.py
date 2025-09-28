from ros2cli.verb import VerbExtension

from ros2tree.api.tree_builder import TreeBuilder
from ros2tree.api.tree_formatter import TreeFormatter


class ServicesVerb(VerbExtension):
    """Display ROS2 services in tree view."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "--no-unicode", action="store_true", help="Use ASCII characters instead of Unicode for tree display"
        )
        parser.add_argument("--connections", action="store_true", help="Show service server and client connections")
        parser.add_argument(
            "--show-prefixes", action="store_true", help="Show type prefixes (service:, ns:) for better grep filtering"
        )
        parser.add_argument(
            "--verbose",
            "-v",
            action="store_true",
            help="Show detailed information: servers and clients for each service",
        )

    def main(self, *, args, parser):
        builder = TreeBuilder()
        formatter = TreeFormatter(use_unicode=not args.no_unicode, show_prefixes=args.show_prefixes)

        try:
            # Get service tree with types if verbose
            service_tree = builder.get_service_tree(include_types=args.verbose)

            # Get connections if requested or verbose enabled
            connections = None
            if args.connections or args.verbose:
                connections = builder.get_service_connections()

            # Format and display
            output = formatter.format_service_tree(
                service_tree,
                show_types=args.verbose,
                show_connections=args.connections or args.verbose,
                connections=connections,
            )
            print(output)

        except (RuntimeError, ValueError, OSError) as e:
            print(f"Error retrieving services: {e}")
            return 1
        finally:
            builder.cleanup()

        return 0
