#!/usr/bin/env python3
"""Install one pinned OpenCV Zoo model atomically; never upgrade ROS's Python stack."""

import argparse
import hashlib
from pathlib import Path
import tempfile
import sys
from urllib.request import urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lekiwi_rmf.object_model import MODEL_NAME, MODEL_SHA256, MODEL_SIZE, MODEL_URL


def install(destination):
    destination = Path(destination).expanduser()
    destination.mkdir(parents=True, exist_ok=True)
    target = destination / MODEL_NAME
    if target.exists():
        with target.open('rb') as stream:
            if hashlib.file_digest(stream, 'sha256').hexdigest() != MODEL_SHA256:
                raise ValueError('existing model has the wrong hash; remove it explicitly before reinstalling')
        return target
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=destination, delete=False) as stream:
            temporary = Path(stream.name)
            size = 0
            with urlopen(MODEL_URL, timeout=30) as response:
                while block := response.read(1024 * 1024):
                    size += len(block)
                    if size > MODEL_SIZE:
                        raise ValueError('download exceeds the pinned model size')
                    stream.write(block)
        with temporary.open('rb') as stream:
            if size != MODEL_SIZE or hashlib.file_digest(stream, 'sha256').hexdigest() != MODEL_SHA256:
                raise ValueError('download does not match the pinned model')
        temporary.replace(target)
        return target
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination', default=str(Path.home() / '.ros/lekiwi/models'))
    args = parser.parse_args()
    print(install(args.destination))


if __name__ == '__main__':
    main()
