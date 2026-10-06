#!/usr/bin/env python3
"""Seal or verify a built release; never change running services or approve motion."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def inventory(release):
    source = release / "source"
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    if subprocess.check_output(["git", "-C", str(source), "status", "--porcelain"], text=True).strip():
        raise ValueError("release source is dirty")
    tracked = subprocess.check_output(["git", "-C", str(source), "ls-files", "-z"]).decode().split("\0")
    sources = {name: digest(source / name) for name in tracked if name}
    installed = {str(path.relative_to(release)): digest(path)
                 for path in sorted((release / "install").rglob("*"))
                 if path.is_file() and "__pycache__" not in path.parts and path.suffix not in (".pyc", ".pyo")}
    if not installed or not (release / "install/lekiwi_rmf/share/lekiwi_rmf/package.xml").is_file():
        raise ValueError("release package is not installed")
    if (release / "install/lekiwi_rmf/.lekiwi-source-revision").read_text().strip() != revision:
        raise ValueError("installed package revision does not match release source")
    if (release / "install/.lekiwi-native-revision").read_text().strip() != revision:
        raise ValueError("native overlay revision does not match release source")
    for name in (
        "rclcpp/lib/librclcpp.so", "class_loader/lib/libclass_loader.so",
        "nav2_lifecycle_manager/lib/nav2_lifecycle_manager/lifecycle_manager",
        "rviz_ogre_vendor/opt/rviz_ogre_vendor/lib/OGRE/RenderSystem_GL.so",
    ):
        path = release / "install" / name
        if not path.is_file() or path.stat().st_size == 0:
            raise ValueError(f"native release artifact is missing: {name}")
    report = release / "build/lekiwi_rmf/release-ctest.xml"
    result = ET.parse(report).getroot()
    expected = {f"test_{path.name}" if path.name.endswith("_launch.py") else path.stem
                for suffix in ("py", "cpp") for path in (source / "test").glob(f"test_*.{suffix}")}
    cases = list(result.iter("testcase"))
    if not expected or {case.get("name") for case in cases} != expected or any(
        case.find(tag) is not None for case in cases for tag in ("failure", "error", "skipped")
    ) or len(cases) != len(expected):
        raise ValueError("release qualification requires every source test to pass")
    evidence = {str(path.relative_to(release)): digest(path)
                for path in sorted((release / "build/lekiwi_rmf/test_results").rglob("*.xml"))}
    evidence[str(report.relative_to(release))] = digest(report)
    settings = source / ".env"
    return {"schema_version": 1, "revision": revision, "source": sources,
            "install": installed, "test_results": evidence,
            "settings": digest(settings) if settings.is_file() else None}


def check_release(release, revision, role, seal=False):
    release = release.resolve()
    actual = {**inventory(release), "role": role}
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
