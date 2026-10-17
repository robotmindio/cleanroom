#!/usr/bin/env python3
"""Install pinned recognizer artifacts atomically; never upgrade ROS's Python stack."""

import argparse
import hashlib
from pathlib import Path
import tempfile
import sys
import os
import subprocess
import yaml
from urllib.request import urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lekiwi_rmf.object_recognition import model_spec, verify_model
from lekiwi_rmf.object_cameras import config_directory


def install(destination, name='yolox'):
    spec = model_spec(name)
    destination = Path(destination).expanduser()
    destination.mkdir(parents=True, exist_ok=True)
    for filename, expected in spec['files'].items():
        target = destination / filename
        if target.exists():
            with target.open('rb') as stream:
                if target.stat().st_size != expected['size'] or hashlib.file_digest(stream, 'sha256').hexdigest() != expected['sha256']:
                    raise ValueError('existing model has wrong hash/size; remove it explicitly before reinstalling')
            continue
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=destination, delete=False) as stream:
                temporary = Path(stream.name)
                size = 0
                with urlopen(spec['base_url'] + '/' + filename, timeout=60) as response:
                    while block := response.read(1024 * 1024):
                        size += len(block)
                        if size > expected['size']:
                            raise ValueError('download exceeds the pinned model size')
                        stream.write(block)
            with temporary.open('rb') as stream:
                if size != expected['size'] or hashlib.file_digest(stream, 'sha256').hexdigest() != expected['sha256']:
                    raise ValueError('download does not match the pinned model')
            temporary.replace(target)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    verify_model(destination, spec)
    return destination


def install_runtime():
    runtime = Path.home() / '.ros/lekiwi/perception-venv'
    environment = {**os.environ, 'PYTHONNOUSERSITE': '1'}
    if not (runtime / 'bin/python').exists():
        subprocess.run(['/usr/bin/python3', '-m', 'venv', '--system-site-packages', str(runtime)], check=True)
    python = str(runtime / 'bin/python')
    subprocess.run([python, '-m', 'pip', 'install', '-r',
                    str(config_directory() / 'requirements-perception.txt')], check=True, env=environment)
    subprocess.run([python, '-m', 'pip', 'install', '--index-url', 'https://download.pytorch.org/whl/cpu',
                    'torch==2.7.0', 'torchvision==0.22.0'], check=True, env=environment)
    subprocess.run([python, '-c', 'import torch, torchvision, transformers; print(torch.__version__, torchvision.__version__, transformers.__version__)'],
                   check=True, env=environment)
    return runtime


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    selected = yaml.safe_load((config_directory() / 'object_perception.yaml').read_text())['/**']['ros__parameters']['model_name']
    parser.add_argument('--name', default=selected, choices=['yolox', 'rtdetr_v2_r18', 'grounding_dino_tiny'])
    parser.add_argument('--destination')
    args = parser.parse_args()
    destination = args.destination or str(Path.home() / '.ros/lekiwi/models' / args.name)
    print(install(destination, args.name))
    if args.name != 'yolox':
        print('runtime:', install_runtime())


if __name__ == '__main__':
    main()
