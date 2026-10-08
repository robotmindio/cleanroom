#!/usr/bin/env python3
"""Seal or verify a built release; never change running services or approve motion."""

import argparse
import hashlib
import json
from pathlib import Path
import stat
import subprocess
import sys
import xml.etree.ElementTree as ET


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def check_safety_acceptance(source):
    """The release's tracked acceptance record must still fit its Nav2 and stow configuration."""
    sys.path.insert(0, str(source))
    try:
        from lekiwi_rmf.safety_acceptance import validate_tracked_acceptance
    finally:
        sys.path.remove(str(source))
    valid, detail = validate_tracked_acceptance(source / "config")
    if not valid:
        raise ValueError(f"tracked safety acceptance does not match the release configuration: {detail}")


def files(root):
    return {str(path.relative_to(root)): {"sha256": digest(path), "mode": stat.S_IMODE(path.stat().st_mode)}
            for path in sorted(root.rglob("*"))
            if path.is_file() and "__pycache__" not in path.parts and path.suffix not in (".pyc", ".pyo")}


def dependency_overlay(release):
    """The shared dependency overlay a release was built on, or None for a self-contained release."""
    marker = release / "install/.lekiwi-overlay"
    if not marker.is_file():
        return None
    overlay = Path(marker.read_text().strip())
    if not overlay.is_absolute() or overlay.parent != release.parent.parent / "overlays":
        raise ValueError("release names a dependency overlay outside its workspace")
    if not (overlay / ".complete").is_file():
        raise ValueError("release dependency overlay is incomplete")
    return overlay


def inventory(release, role):
    source = release / "source"
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    if subprocess.check_output(["git", "-C", str(source), "status", "--porcelain"], text=True).strip():
        raise ValueError("release source is dirty")
    check_safety_acceptance(source)
    tracked = subprocess.check_output(["git", "-C", str(source), "ls-files", "-z"]).decode().split("\0")
    sources = {name: digest(source / name) for name in tracked if name}
    installed = {f"install/{name}": value for name, value in files(release / "install").items()}
    overlay = dependency_overlay(release)
    # A reused overlay was built by an earlier revision; its exact files are part of this release.
    overlay_files = files(overlay / "install") if overlay else None
    native_root = (overlay or release) / "install"
    if not installed or not (release / "install/lekiwi_rmf/share/lekiwi_rmf/package.xml").is_file():
        raise ValueError("release package is not installed")
    if (release / "install/lekiwi_rmf/.lekiwi-source-revision").read_text().strip() != revision:
        raise ValueError("installed package revision does not match release source")
    if role == "compute" and overlay is None and (
            release / "install/.lekiwi-native-revision").read_text().strip() != revision:
        raise ValueError("native overlay revision does not match release source")
    native_artifacts = (
        "rclcpp/lib/librclcpp.so", "class_loader/lib/libclass_loader.so",
        "nav2_lifecycle_manager/lib/nav2_lifecycle_manager/lifecycle_manager",
        "rviz_ogre_vendor/opt/rviz_ogre_vendor/lib/OGRE/RenderSystem_GL.so",
    ) if role == "compute" else ()
    for name in native_artifacts:
        path = native_root / name
        if not path.is_file() or path.stat().st_size == 0:
            raise ValueError(f"native release artifact is missing: {name}")
    report = release / "build/lekiwi_rmf/release-ctest.xml"
    result = ET.parse(report).getroot()
    expected = {f"test_{path.name}" if path.name.endswith("_launch.py") else path.stem
                for suffix in ("py", "cpp") for path in (source / "test").glob(f"test_*.{suffix}")}
    if role == "device":
        selected = set((source / "config/device_tests.txt").read_text().split())
        if not selected <= expected:
            raise ValueError("device qualification names a missing source test")
        expected = selected
    cases = list(result.iter("testcase"))
    if not expected or {case.get("name") for case in cases} != expected or any(
        case.find(tag) is not None for case in cases for tag in ("failure", "error", "skipped")
    ) or len(cases) != len(expected):
        raise ValueError("release qualification requires every source test selected for this role to pass")
    python_tests = {f"test_{path.name}" if path.name.endswith("_launch.py") else path.stem
                    for path in (source / "test").glob("test_*.py")} & expected
    for name in sorted(python_tests):
        path = release / "build/lekiwi_rmf/test_results/lekiwi_rmf" / f"{name}.xunit.xml"
        if not path.is_file():
            raise ValueError(f"release qualification is missing Python test evidence: {name}")
        result = ET.parse(path).getroot()
        if not list(result.iter("testcase")) or any(
            list(result.iter(tag)) for tag in ("failure", "error", "skipped")
        ):
            raise ValueError(f"release qualification requires every Python test case to pass: {name}")
    evidence = {str(path.relative_to(release)): digest(path)
                for path in sorted((release / "build/lekiwi_rmf/test_results").rglob("*.xml"))}
    evidence[str(report.relative_to(release))] = digest(report)
    settings = source / ".env"
    return {"schema_version": 2, "revision": revision, "source": sources,
            "install": installed, "test_results": evidence,
            **({"dependency_overlay": {"path": str(overlay), "files": overlay_files}} if overlay else {}),
            "settings": digest(settings) if settings.is_file() else None}


def check_release(release, revision, role, seal=False):
    release = release.resolve()
    actual = {**inventory(release, role), "role": role}
    if actual["revision"] != revision:
        raise ValueError("release revision differs from requested revision")
    manifest = release / "release.json"
    if seal:
        if manifest.exists():
            raise ValueError("refusing to overwrite a sealed release")
        manifest.write_text(json.dumps(actual, indent=2) + "\n", encoding="utf-8")
        manifest.chmod(0o600)
    elif json.loads(manifest.read_text(encoding="utf-8")) != actual:
        raise ValueError("release files or test evidence changed after qualification")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("seal", "verify"))
    parser.add_argument("release", type=Path)
    parser.add_argument("revision")
    parser.add_argument("role", choices=("compute", "device"))
    args = parser.parse_args()
    try:
        check_release(args.release, args.revision, args.role, args.action == "seal")
    except (OSError, ValueError, ET.ParseError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"release check failed: {error}\n")


if __name__ == "__main__":
    main()
