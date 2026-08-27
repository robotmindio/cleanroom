"""Unit coverage for the fail-closed free_fleet ownership preflight."""

from __future__ import annotations

import pytest

from lekiwi_rmf.rmf_owner_guard import (
    CLEAR_EXIT,
    CONFLICT_EXIT,
    ERROR_EXIT,
    conflicting_free_fleet_nodes,
    expected_free_fleet_nodes,
    fleet_name_from_config,
    qualified_node_name,
    resolve_fleet_name,
)


def test_exit_codes_keep_conflict_distinct_from_a_guard_error():
    assert CLEAR_EXIT == 0
    assert CONFLICT_EXIT != CLEAR_EXIT
    assert ERROR_EXIT not in {CLEAR_EXIT, CONFLICT_EXIT}


def test_expected_nodes_match_the_upstream_free_fleet_adapter_contract():
    assert expected_free_fleet_nodes("lekiwi") == {
        "/lekiwi_command_handle", "/lekiwi_fleet_adapter",
    }


def test_graph_conflict_detection_is_exact_and_namespace_aware():
    graph = [
        ("lekiwi_command_handle", "/"),
        ("lekiwi_fleet_adapter", "/"),
        ("other_fleet_adapter", "/"),
        ("lekiwi_fleet_adapter_extra", "/"),
        ("lekiwi_fleet_adapter", "/another_namespace"),
    ]
    assert conflicting_free_fleet_nodes(graph, "lekiwi") == {
        "/lekiwi_command_handle", "/lekiwi_fleet_adapter",
    }


def test_graph_clear_when_no_exact_ownership_node_is_discovered():
    assert not conflicting_free_fleet_nodes(
        [("lekiwi_fleet_adapter_extra", "/"), ("lekiwi_command_handle", "/ops")],
        "lekiwi",
    )


def test_qualified_node_name_normalizes_root_namespace():
    assert qualified_node_name("node", "/") == "/node"
    assert qualified_node_name("node", "fleet") == "/fleet/node"


@pytest.mark.parametrize("fleet_name", ["", "  ", "bad/name"])
def test_invalid_fleet_names_are_rejected(fleet_name):
    with pytest.raises(ValueError):
        expected_free_fleet_nodes(fleet_name)


def test_fleet_name_is_loaded_from_the_selected_fleet_config(tmp_path):
    config = tmp_path / "fleet.yaml"
    config.write_text("rmf_fleet:\n  name: cleanroom_fleet\n")
    assert fleet_name_from_config(str(config)) == "cleanroom_fleet"
    assert resolve_fleet_name("", str(config)) == "cleanroom_fleet"


def test_config_and_launch_fleet_identity_must_agree(tmp_path):
    config = tmp_path / "fleet.yaml"
    config.write_text("rmf_fleet:\n  name: cleanroom_fleet\n")
    with pytest.raises(ValueError, match="does not match"):
        resolve_fleet_name("other_fleet", str(config))


def test_missing_or_malformed_config_is_a_configuration_error(tmp_path):
    with pytest.raises(ValueError, match="unable to read"):
        fleet_name_from_config(str(tmp_path / "missing.yaml"))
    malformed = tmp_path / "malformed.yaml"
    malformed.write_text("- not-a-fleet-config\n")
    with pytest.raises(ValueError, match="rmf_fleet.name"):
        fleet_name_from_config(str(malformed))


@pytest.mark.parametrize("value", ["null", "true", "1", '""'])
def test_fleet_config_name_must_be_a_nonempty_string(tmp_path, value):
    config = tmp_path / "fleet.yaml"
    config.write_text(f"rmf_fleet:\n  name: {value}\n")
    with pytest.raises(ValueError, match="non-empty string"):
        fleet_name_from_config(str(config))
