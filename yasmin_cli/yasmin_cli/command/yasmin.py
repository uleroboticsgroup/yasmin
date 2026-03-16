from ros2cli.command import CommandExtension

from yasmin_cli.verb.inspect import add_inspect_verb


class YasminCommand(CommandExtension):
    """YASMIN command line tools."""

    def add_arguments(self, parser, cli_name):
        subparsers = parser.add_subparsers(dest="verb", metavar="verb")
        subparsers.required = True

        add_inspect_verb(subparsers)

        self._parser = parser

    def main(self, *, parser, args):
        if hasattr(args, "main"):
            return args.main(args)
        self._parser.print_help()
        return 0
