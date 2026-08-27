#!/usr/bin/env python3
"""Validate one immutable map/RMF bundle before deployment."""

import argparse

from lekiwi_rmf.map_bundle import MapBundleError, validate_map_bundle


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bundle")
    parser.add_argument(
        "--allow-unapproved", action="store_true",
        help="validate a mapping/development artifact without approving it for RMF operation",
    )
    args = parser.parse_args()
    try:
        bundle = validate_map_bundle(args.bundle, require_approved=not args.allow_unapproved)
    except MapBundleError as error:
        parser.error(str(error))
    print(f"validated map bundle {bundle.map_id}: {bundle.navigation_graph}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
