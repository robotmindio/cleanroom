import hashlib
from pathlib import Path

import pytest
import yaml

from lekiwi_rmf.map_bundle import _OccupancyMap, MapBundleError, validate_map_bundle


def test_pgm_rows_are_mirrored_into_occupancy_grid_coordinates():
    grid = _OccupancyMap(
        width=1,
        height=2,
        # Top raster pixel occupied, bottom raster pixel free.
        pixels=(0, 254),
        maximum=255,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        occupied_threshold=0.65,
        free_threshold=0.25,
        negate=False,
    )
    assert grid.cell_free(0, 0)
    assert not grid.cell_free(0, 1)


def _write(path: Path, content: str) -> str:
    path.write_text(content, encoding="utf-8")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _bundle(
    tmp_path: Path, *, approved: bool = True, resolution: float = 0.05,
    blocked_vertex: bool = False, footprint: float = 0.22,
) -> Path:
    pixels = [254] * (60 * 60)
    # World (0, 0) maps to this cell with the origin below.  An occupied cell
    # here must reject both the vertex and the lanes passing through it.
    if blocked_vertex:
        pixels[29 * 60 + 30] = 0
    image = "P2\n60 60\n255\n" + "\n".join(
        " ".join(str(pixel) for pixel in pixels[row * 60:(row + 1) * 60])
        for row in range(60)
    ) + "\n"
    image_hash = _write(tmp_path / "map.pgm", image)
    map_hash = _write(tmp_path / "map.yaml", yaml.safe_dump({
        "image": "map.pgm", "resolution": resolution, "origin": [-1.5, -1.5, 0.0],
        "occupied_thresh": 0.65, "free_thresh": 0.25, "negate": 0,
    }))
    graph_hash = _write(tmp_path / "graph.yaml", yaml.safe_dump({
        "levels": {"L1": {
            "vertices": [[0.0, 0.0, {"name": "a"}], [1.0, 0.0, {"name": "b"}], [1.0, 1.0, {"name": "c"}]],
            "lanes": [[0, 1, {}], [1, 0, {}], [1, 2, {}], [2, 1, {}]],
        }}
    }))
    fleet_hash = _write(tmp_path / "fleet.yaml", yaml.safe_dump({
        "rmf_fleet": {"profile": {"footprint": footprint}},
        "reference_coordinates": {"L1": {
            "rmf": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
            "robot": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
        }}
    }))
    report_hash = _write(tmp_path / "report.md", "accepted\n") if approved else None
    manifest = tmp_path / "bundle.yaml"
    manifest.write_text(yaml.safe_dump({
        "schema_version": 1,
        "map_id": "fixture",
        "validated": approved,
        "robot_footprint_radius": 0.22,
        "artifacts": {
            "occupancy_yaml": {"path": "map.yaml", "sha256": map_hash},
            "occupancy_image": {"path": "map.pgm", "sha256": image_hash},
            "navigation_graph": {"path": "graph.yaml", "sha256": graph_hash},
            "fleet_config": {"path": "fleet.yaml", "sha256": fleet_hash},
            **({
                "validation_report": {"path": "report.md", "sha256": report_hash},
            } if approved else {}),
        },
        "validation": {
            "maximum_resolution": 0.05,
            "maximum_reference_residual": 0.02,
        },
    }), encoding="utf-8")
    return manifest


def test_valid_approved_bundle_resolves_immutable_artifacts(tmp_path):
    result = validate_map_bundle(_bundle(tmp_path))
    assert result.map_id == "fixture"
    assert result.navigation_graph == tmp_path / "graph.yaml"


def test_checksum_mismatch_is_rejected(tmp_path):
    bundle = _bundle(tmp_path)
    (tmp_path / "graph.yaml").write_text("changed", encoding="utf-8")
    with pytest.raises(MapBundleError, match="checksum mismatch"):
        validate_map_bundle(bundle)


def test_approved_validation_report_is_checksum_pinned(tmp_path):
    bundle = _bundle(tmp_path)
    (tmp_path / "report.md").write_text("replaced after approval\n", encoding="utf-8")
    with pytest.raises(MapBundleError, match="validation_report checksum mismatch"):
        validate_map_bundle(bundle)


def test_unapproved_bundle_cannot_run_rmf(tmp_path):
    with pytest.raises(MapBundleError, match="not passed deployment acceptance"):
        validate_map_bundle(_bundle(tmp_path, approved=False))


def test_development_bundle_can_be_inspected_but_resolution_is_still_validated(tmp_path):
    bundle = _bundle(tmp_path, approved=False, resolution=0.5)
    with pytest.raises(MapBundleError, match="resolution"):
        validate_map_bundle(bundle, require_approved=False)


def test_reference_residual_fits_translation_and_rotation(tmp_path):
    bundle = _bundle(tmp_path)
    fleet_path = tmp_path / "fleet.yaml"
    fleet = yaml.safe_load(fleet_path.read_text(encoding="utf-8"))
    # Robot points are a rigidly transformed copy of the RMF points.  A direct
    # point difference would be large, while the fitted SE(2) residual is zero.
    fleet["reference_coordinates"]["L1"]["robot"] = [
        [2.0, -1.0], [2.0, 0.0], [1.0, 0.0]
    ]
    fleet_path.write_text(yaml.safe_dump(fleet), encoding="utf-8")
    # The artifact digest must describe the edited fleet file.
    manifest_path = bundle
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    manifest["artifacts"]["fleet_config"]["sha256"] = hashlib.sha256(
        fleet_path.read_bytes()
    ).hexdigest()
    manifest_path.write_text(yaml.safe_dump(manifest), encoding="utf-8")
    assert validate_map_bundle(bundle).map_id == "fixture"


def test_reference_residual_rejects_non_rigid_correspondence(tmp_path):
    bundle = _bundle(tmp_path)
    fleet_path = tmp_path / "fleet.yaml"
    fleet = yaml.safe_load(fleet_path.read_text(encoding="utf-8"))
    fleet["reference_coordinates"]["L1"]["robot"][2][0] = 1.1
    fleet_path.write_text(yaml.safe_dump(fleet), encoding="utf-8")
    manifest = yaml.safe_load(bundle.read_text(encoding="utf-8"))
    manifest["artifacts"]["fleet_config"]["sha256"] = hashlib.sha256(
        fleet_path.read_bytes()
    ).hexdigest()
    bundle.write_text(yaml.safe_dump(manifest), encoding="utf-8")
    with pytest.raises(MapBundleError, match="reference-coordinate residual"):
        validate_map_bundle(bundle)


def test_occupied_or_unknown_graph_space_is_rejected(tmp_path):
    with pytest.raises(MapBundleError, match="occupied/unknown"):
        validate_map_bundle(_bundle(tmp_path, blocked_vertex=True))


def test_one_way_graph_that_can_strand_robot_is_rejected(tmp_path):
    bundle = _bundle(tmp_path)
    graph_path = tmp_path / "graph.yaml"
    graph = yaml.safe_load(graph_path.read_text(encoding="utf-8"))
    graph["levels"]["L1"]["lanes"] = [[0, 1, {}], [1, 2, {}]]
    graph_path.write_text(yaml.safe_dump(graph), encoding="utf-8")
    manifest = yaml.safe_load(bundle.read_text(encoding="utf-8"))
    manifest["artifacts"]["navigation_graph"]["sha256"] = hashlib.sha256(
        graph_path.read_bytes()
    ).hexdigest()
    bundle.write_text(yaml.safe_dump(manifest), encoding="utf-8")
    with pytest.raises(MapBundleError, match="not strongly connected"):
        validate_map_bundle(bundle)


def test_fleet_footprint_must_match_bundle_footprint(tmp_path):
    with pytest.raises(MapBundleError, match="fleet footprint"):
        validate_map_bundle(_bundle(tmp_path, footprint=0.30))


def test_bundle_circle_must_enclose_optional_navigation_polygon(tmp_path):
    bundle = _bundle(tmp_path)
    nav_path = tmp_path / "nav2.yaml"
    nav_hash = _write(nav_path, yaml.safe_dump({
        "local_costmap": {"ros__parameters": {
            "footprint": "[[0.16, 0.16], [0.16, -0.16], [-0.16, -0.16]]",
        }},
    }))
    manifest = yaml.safe_load(bundle.read_text(encoding="utf-8"))
    manifest["artifacts"]["nav2_params"] = {
        "path": "nav2.yaml", "sha256": nav_hash,
    }
    bundle.write_text(yaml.safe_dump(manifest), encoding="utf-8")

    with pytest.raises(MapBundleError, match="does not enclose"):
        validate_map_bundle(bundle)
