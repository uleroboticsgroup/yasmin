from yasmin_editor.plugins_manager.plugin_manager import PluginManager


def _plugin_id(plugin) -> str:
    if plugin.plugin_type == "python":
        return f"{plugin.module}.{plugin.class_name}"
    if plugin.plugin_type == "cpp":
        return plugin.class_name or ""
    if plugin.plugin_type == "xml":
        if plugin.package_name:
            return f"{plugin.package_name}/{plugin.file_name}"
        return plugin.file_name or ""
    return ""


def _plugin_summary(plugin) -> str:
    return (
        f"[{plugin.plugin_type:<6}] {_plugin_id(plugin)} | "
        f"outcomes={len(plugin.outcomes)}, "
        f"inputs={len(plugin.input_keys)}, "
        f"outputs={len(plugin.output_keys)}"
    )


def _print_key_block(title: str, keys: list[dict]) -> None:
    if not keys:
        return
    print(f"{title}:")

    for key in keys:
        name = key.get("name", "")
        description = key.get("description", "")
        has_default = key.get("has_default", False)

        line = f"  - {name}"
        if has_default:
            default_value = key.get("default_value")
            default_type = key.get("default_value_type", "unknown")
            line += f" [default={default_value!r}, type={default_type}]"
        print(line)

        if description:
            print(f"      {description}")


def _print_plugin_details(plugin) -> None:
    print(f"Plugin:      {_plugin_id(plugin)}")
    print(f"Type:        {plugin.plugin_type}")
    print(f"Description: {plugin.description if plugin.description else '-'}")
    print(f"Outcomes:    {', '.join(plugin.outcomes) if plugin.outcomes else '-'}")
    _print_key_block("Input keys", plugin.input_keys)
    _print_key_block("Output keys", plugin.output_keys)


def add_inspect_verb(subparsers):
    parser = subparsers.add_parser(
        "inspect",
        help="Inspect YASMIN plugins and metadata",
        description="Inspect discovered YASMIN states, XML state machines, and metadata",
    )

    parser.add_argument(
        "--type",
        choices=["all", "cpp", "python", "xml"],
        default="all",
        help="Filter plugin type",
    )
    parser.add_argument(
        "--search",
        default=None,
        help="Filter by substring",
    )
    parser.add_argument(
        "--plugin",
        default=None,
        help="Show one exact plugin id",
    )
    parser.add_argument(
        "--show-all",
        action="store_true",
        help="Show detailed metadata for all matching plugins",
    )

    parser.set_defaults(main=_main_inspect)


def _main_inspect(args):
    manager = PluginManager()
    manager.load_all_plugins()

    plugins = []

    if args.type in ("all", "cpp"):
        plugins.extend(manager.cpp_plugins)
    if args.type in ("all", "python"):
        plugins.extend(manager.python_plugins)
    if args.type in ("all", "xml"):
        plugins.extend(manager.xml_files)

    if args.search:
        search = args.search.lower()
        filtered = []
        for plugin in plugins:
            haystack = [
                _plugin_id(plugin),
                plugin.description or "",
                " ".join(plugin.outcomes),
                " ".join(key.get("name", "") for key in plugin.input_keys),
                " ".join(key.get("name", "") for key in plugin.output_keys),
            ]
            if any(search in entry.lower() for entry in haystack):
                filtered.append(plugin)
        plugins = filtered

    plugins = sorted(plugins, key=_plugin_id)

    if args.plugin:
        plugin = next((p for p in plugins if _plugin_id(p) == args.plugin), None)
        if plugin is None:
            print(f"Plugin not found: {args.plugin}")
            return 1
        _print_plugin_details(plugin)
        return 0

    if args.show_all:
        if not plugins:
            print("No plugins found.")
            return 0

        for index, plugin in enumerate(plugins):
            if index > 0:
                print()
                print("-" * 80)
                print()
            _print_plugin_details(plugin)
        return 0

    if not plugins:
        print("No plugins found.")
        return 0

    for plugin in plugins:
        print(_plugin_summary(plugin))
    return 0
