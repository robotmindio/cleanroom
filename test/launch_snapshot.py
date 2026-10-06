"""Resolve bringup.launch.py into a normalized, process-free description.

The resolver walks the launch description the way ``launch`` itself would,
but records Node/ExecuteProcess actions instead of starting them.  Package
includes owned by this repository are flattened (so splitting bringup into
several files is invisible), external includes are recorded with their
resolved arguments, and readiness-gate handlers are expanded into the stage
they start.  Parameters are reported as each node's effective, merged
parameter set: parameter files are read and their matching sections applied
in order, exactly as rcl does, so layering YAML files is also invisible.
"""

from __future__ import annotations

import contextlib
import functools
import json
import os
from pathlib import Path
import re
import runpy
import tempfile
import types
from unittest import mock

import yaml

ROOT = Path(__file__).resolve().parents[1]
LD06_PREFIX = "/dev/serial/by-id/usb-Silicon_Labs_CP2102"
# Launch files that are recorded as includes instead of flattened: MoveIt
# builds a full robot model from installed packages and has its own tests.
RECORDED_PACKAGE_INCLUDES = {"moveit.launch.py"}


def _fake_share(prefix: Path) -> Path:
    index = prefix / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True)
    (index / "lekiwi_rmf").write_text("")
    share = prefix / "share" / "lekiwi_rmf"
    share.mkdir()
    for name in ("config", "launch", "maps", "urdf", "worlds", "scripts", ".setup_assistant"):
        (share / name).symlink_to(ROOT / name)
    return share


class _Normalizer:
    def __init__(self, prefix: Path, home: Path):
        from ament_index_python.packages import get_packages_with_prefixes

        replacements = {str(prefix / "share" / "lekiwi_rmf"): "$(share lekiwi_rmf)",
                        str(prefix): "$(prefix lekiwi_rmf)", str(home): "$(env HOME)"}
        for package, package_prefix in get_packages_with_prefixes().items():
            if package != "lekiwi_rmf":
                replacements.setdefault(
                    str(Path(package_prefix) / "share" / package), f"$(share {package})")
        self._replacements = sorted(replacements.items(), key=lambda item: -len(item[0]))
        self._temporary = tempfile.gettempdir() + os.sep
        self._prefix = str(prefix)

    def __call__(self, value):
        if isinstance(value, dict):
            return {str(key): self(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [self(item) for item in value]
        if isinstance(value, Path):
            value = str(value)
        if not isinstance(value, str):
            return value
        # A generated file (RewrittenYaml) is identified by its content.
        if (value.startswith(self._temporary) and not value.startswith(self._prefix)
                and os.path.isfile(value)):
            return {"generated_yaml": self(yaml.safe_load(Path(value).read_text()))}
        for old, new in self._replacements:
            value = value.replace(old, new)
        return value


def _perform(context, value):
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

    if value is None:
        return None
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def _flatten(prefix: str, value, output: dict):
    if isinstance(value, dict):
        for key, item in value.items():
            _flatten(f"{prefix}{key}.", item, output)
    else:
        output[prefix[:-1]] = list(value) if isinstance(value, tuple) else value


def _matches(pattern: str, fully_qualified_name: str) -> bool:
    pattern = pattern if pattern.startswith("/") else "/" + pattern
    expression = "".join(
        ".*" if part == "**" else "[^/]*" if part == "*" else re.escape(part)
        for part in re.split(r"(\*\*|\*)", pattern))
    return re.fullmatch(expression, fully_qualified_name) is not None


def effective_parameters(context, parameters, name: str | None, namespace: str = "") -> dict:
    """Merge parameter files and dictionaries in order, as rcl applies them."""
    from launch_ros.utilities import evaluate_parameters

    fully_qualified = f"{namespace.rstrip('/')}/{name}" if name else None
    merged: dict = {}
    for item in evaluate_parameters(context, parameters):
        if isinstance(item, Path):
            document = yaml.safe_load(item.read_text()) or {}
            for pattern, section in document.items():
                if pattern in ("/**", "**") or (
                        fully_qualified and _matches(str(pattern), fully_qualified)):
                    _flatten("", section.get("ros__parameters", {}), merged)
        else:
            _flatten("", dict(item), merged)
    return merged


class _Resolver:
    def __init__(self, context, normalize, base_environment):
        self.context = context
        self.normalize = normalize
        self.base_environment = base_environment
        self.declared: dict = {}

    @staticmethod
    def reachable(records: list) -> list:
        """Drop handlers whose gate is never started in this configuration."""
        def started(items):
            for record in items:
                yield record.get("_id")
                yield from started(record.get("start", []))

        def prune(items, launched):
            return [{**record, "start": prune(record["start"], launched)} if "start" in record
                    else record for record in items
                    if record.get("_target", None) is None or record["_target"] in launched]

        while True:
            pruned = prune(records, set(started(records)))
            if pruned == records:
                break
            records = pruned

        def strip(items):
            return [{key: strip(value) if key in ("start", "on_failure") else value
                     for key, value in record.items() if key not in ("_id", "_target")}
                    for record in items]

        # Where a handler is registered does not matter, only whether its gate
        # starts: list every reachable handler at the top level.
        def hoist(items, handlers):
            kept = []
            for record in items:
                if "start" in record:
                    record = {**record, "start": hoist(record["start"], handlers)}
                    handlers.append(record)
                else:
                    kept.append(record)
            return kept

        handlers = []
        top = hoist(strip(records), handlers)
        return top + handlers

    def _launch_environment(self):
        return {key: value for key, value in os.environ.items()
                if self.base_environment.get(key) != value}

    def _process_common(self, action):
        from launch.utilities.type_utils import perform_typed_substitution

        description = action.process_description
        environment = self._launch_environment()
        for key, value in description.additional_env or []:
            environment[_perform(self.context, key)] = _perform(self.context, value)
        output = action._ExecuteLocal__output
        respawn = action._ExecuteLocal__respawn
        return {
            "env": environment,
            "respawn": perform_typed_substitution(self.context, respawn, bool),
            "respawn_delay": action._ExecuteLocal__respawn_delay,
            "output": output if isinstance(output, dict) else _perform(self.context, output),
        }

    def node_name(self, node):
        return _perform(self.context, node._Node__node_name)

    def node(self, node):
        name = self.node_name(node)
        namespace = _perform(self.context, node._Node__node_namespace) or ""
        return {
            "node": f"{_perform(self.context, node.node_package)}/"
                    f"{_perform(self.context, node.node_executable)}",
            "name": name,
            "namespace": namespace,
            "arguments": [_perform(self.context, item) for item in node._Node__arguments or []],
            "ros_arguments": [_perform(self.context, item) for item in node._Node__ros_arguments or []],
            "remappings": [[_perform(self.context, source), _perform(self.context, target)]
                           for source, target in node._Node__remappings],
            "parameters": effective_parameters(
                self.context, node._Node__parameters, name, namespace),
            **self._process_common(node),
        }

    def process(self, action):
        return {
            "process": [_perform(self.context, part) for part in action.process_description.cmd],
            "name": _perform(self.context, action.process_description.name),
            **self._process_common(action),
        }

    def walk(self, entities) -> list:
        records = []
        for entity in entities or []:
            records.extend(self.visit(entity))
        return records

    def visit(self, entity) -> list:
        from launch import Action
        from launch.actions import (
            DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler,
            SetEnvironmentVariable)
        from launch_ros.actions import Node

        if isinstance(entity, Action) and entity.condition is not None and \
                not entity.condition.evaluate(self.context):
            return []
        if isinstance(entity, Node):
            return [{"_id": id(entity), **self.node(entity)}]
        if isinstance(entity, ExecuteProcess):
            return [self.process(entity)]
        if isinstance(entity, LogInfo):
            return [{"log": _perform(self.context, entity.msg)}]
        if isinstance(entity, IncludeLaunchDescription):
            location = Path(_perform(
                self.context, entity.launch_description_source._LaunchDescriptionSource__location))
            if location.parent.name == "launch" and location.parent.parent.name == "lekiwi_rmf" \
                    and location.name not in RECORDED_PACKAGE_INCLUDES:
                return self.walk(entity.visit(self.context))
            return [{"include": str(location), "arguments": {
                _perform(self.context, key): _perform(self.context, value)
                for key, value in entity.launch_arguments}}]
        if isinstance(entity, RegisterEventHandler):
            handler = entity.event_handler
            target = handler._OnActionEventBase__action_matcher
            on_exit = handler._OnActionEventBase__on_event
            started = on_exit(types.SimpleNamespace(returncode=0, action=target), self.context)
            failed = on_exit(types.SimpleNamespace(returncode=1, action=target), self.context)
            return [{
                "_target": id(target),
                "after": self.node_name(target) if isinstance(target, Node) else str(target),
                "start": self.walk(started),
                "on_failure": self.walk(failed),
            }]
        if isinstance(entity, DeclareLaunchArgument):
            self.declared[entity.name] = {
                "default": None if entity.default_value is None else _perform(self.context, entity.default_value),
                "choices": entity.choices,
            }
        if isinstance(entity, SetEnvironmentVariable):
            entity.visit(self.context)
            return []
        return self.walk(entity.visit(self.context))


def _canonical(records: list) -> list:
    """Order-independent form: launch starts every listed action concurrently."""
    for record in records:
        for key in ("start", "on_failure"):
            if key in record:
                record[key] = _canonical(record[key])
    return sorted(records, key=lambda record: json.dumps(record, sort_keys=True))


@contextlib.contextmanager
def _patched_environment(tmp: Path, serial_devices, map_bundle_approved: bool):
    from launch.substitutions import Command
    import lekiwi_rmf.map_bundle as map_bundle

    prefix = tmp / "prefix"
    share = _fake_share(prefix)
    home = tmp / "home"
    home.mkdir()
    real_exists = os.path.exists

    def exists(path):
        if isinstance(path, str) and path.startswith(LD06_PREFIX):
            return path in serial_devices
        return real_exists(path)

    def symbolic_command(self, context):
        from launch.utilities import perform_substitutions

        return f"$(command {perform_substitutions(context, self.command)})"

    def approved_bundle(path, require_approved):
        return types.SimpleNamespace(
            occupancy_yaml=share / "maps" / "cleanroom.yaml",
            navigation_graph=share / "maps" / "nav_graph.yaml",
            fleet_config=share / "config" / "fleet_config.yaml",
        )

    environment = {"HOME": str(home),
                   "AMENT_PREFIX_PATH": f"{prefix}{os.pathsep}{os.environ.get('AMENT_PREFIX_PATH', '')}"}
    with contextlib.ExitStack() as stack:
        stack.enter_context(mock.patch.dict(os.environ, environment))
        for name in ("GZ_SIM_RESOURCE_PATH", "GZ_SIM_SYSTEM_PLUGIN_PATH"):
            os.environ.pop(name, None)
        stack.enter_context(mock.patch("os.path.exists", exists))
        stack.enter_context(mock.patch.object(Command, "perform", symbolic_command))
        stack.enter_context(mock.patch(
            "lekiwi_rmf.launch_validation._tailscale_ipv4_addresses", lambda: frozenset()))
        if map_bundle_approved:
            stack.enter_context(mock.patch.object(map_bundle, "validate_map_bundle", approved_bundle))
        yield prefix, home


def resolve_launch(*, serial_devices=(), map_bundle_approved: bool = False,
                   launch_file: str = "bringup.launch.py", **arguments) -> tuple[list, dict]:
    """Resolve ``ros2 launch lekiwi_rmf <launch_file> <arguments>`` without starting anything.

    Returns the normalized process records and the final launch configurations.
    ``serial_devices`` lists the /dev/serial/by-id LD06 paths that exist.
    """
    from launch import LaunchContext

    with tempfile.TemporaryDirectory() as directory, \
            _patched_environment(Path(directory), serial_devices, map_bundle_approved) as (prefix, home):
        normalize = _Normalizer(prefix, home)
        base_environment = dict(os.environ)
        module = runpy.run_path(str(prefix / "share" / "lekiwi_rmf" / "launch" / launch_file))
        description = module["generate_launch_description"]()
        context = LaunchContext()
        context.launch_configurations.update({key: str(value) for key, value in arguments.items()})
        resolver = _Resolver(context, normalize, base_environment)
        records = resolver.reachable(resolver.walk(description.entities))
        return _canonical(normalize(records)), normalize(dict(context.launch_configurations))


def declared_arguments(**arguments) -> dict:
    """Every launch argument bringup (and its flattened includes) declares."""
    from launch import LaunchContext

    with tempfile.TemporaryDirectory() as directory, \
            _patched_environment(Path(directory), (), False) as (prefix, home):
        module = runpy.run_path(str(prefix / "share" / "lekiwi_rmf" / "launch" / "bringup.launch.py"))
        context = LaunchContext()
        context.launch_configurations.update(arguments)
        resolver = _Resolver(context, _Normalizer(prefix, home), dict(os.environ))
        resolver.walk(module["generate_launch_description"]().entities)
        return _Normalizer(prefix, home)(resolver.declared)


@functools.lru_cache(maxsize=None)
def _cached(key: str) -> str:
    return json.dumps(resolve_launch(**json.loads(key))[0])


def resolve_bringup(**arguments) -> list:
    """Normalized bringup records; cached because the resolution is deterministic."""
    return json.loads(_cached(json.dumps(arguments, sort_keys=True)))


def find_nodes(records: list, *, name: str | None = None, node: str | None = None) -> list:
    """Every node record, including those started by readiness gates."""
    found = []
    for record in records:
        if "node" in record and (name is None or record["name"] == name) and (
                node is None or record["node"] == node):
            found.append(record)
        found.extend(find_nodes(record.get("start", []), name=name, node=node))
    return found


def find_node(records: list, **criteria) -> dict:
    nodes = find_nodes(records, **criteria)
    assert len(nodes) == 1, f"expected exactly one node matching {criteria}, found {len(nodes)}"
    return nodes[0]


def gate_stage(records: list, gate: str) -> dict:
    """The record of what the named readiness gate starts on success."""
    for record in records:
        if record.get("after") == gate:
            return record
        try:
            return gate_stage(record.get("start", []), gate)
        except LookupError:
            pass
    raise LookupError(gate)
