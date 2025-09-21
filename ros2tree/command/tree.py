from ros2cli.command import CommandExtension, add_subparsers_on_demand


class TreeCommand(CommandExtension):
    """Display ROS2 topics and nodes in tree view."""

    def add_arguments(self, parser, cli_name):
        self._subparser = parser
        add_subparsers_on_demand(parser, cli_name, "_verb", "ros2tree.verb", required=False)

    def main(self, *, parser, args):
        if not hasattr(args, "_verb"):
            self._subparser.print_help()
            return 0
        extension = getattr(args, "_verb")
        return extension.main(args=args, parser=parser)
