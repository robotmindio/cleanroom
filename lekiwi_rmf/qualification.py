"""Stored qualification suites: what qualified a skill or recognizer, re-run before a new version replaces it.

`config/qualification_suites.yaml` is tracked. Each suite names a frozen protocol (a skill's
evaluation protocol, or a recognizer's held-out manifest and labels) and the content-addressed
report that last passed it, kept under ~/.ros/lekiwi/qualification-suites/<sha256>.json.
Promotion (scripts/register-skill.py, scripts/register-recognizer.py) requires a new passing
report for every suite of the version being replaced, and then records the new reports as the
suites' baselines, so the set only grows and never silently drops a previously qualified case.
"""

import hashlib
import json
import os
from pathlib import Path
import re
import shutil

import yaml

from lekiwi_rmf.object_cameras import config_directory

STORE = Path.home() / '.ros/lekiwi/qualification-suites'
NAME = re.compile(r'^[a-z0-9][a-z0-9_.-]{0,63}$')
SHA256 = re.compile(r'^[0-9a-f]{64}$')
SKILL_SUITE = {'name', 'protocol', 'protocol_sha256', 'qualified_version', 'report_sha256'}
RECOGNIZER_SUITE = {'name', 'manifest', 'manifest_sha256', 'split', 'labels', 'qualified_model', 'report_sha256'}


def canonical_sha256(document):
    return hashlib.sha256(json.dumps(document, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def default_path():
    return config_directory() / 'qualification_suites.yaml'


def load_suites(path=None):
    """Every stored suite, validated; one malformed suite rejects the file."""
    path = Path(path or default_path())
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict) or set(document) != {'skills', 'recognizers'}:
        raise ValueError('qualification suites need exactly `skills` and `recognizers`')
    for kind, fields in (('skills', SKILL_SUITE), ('recognizers', RECOGNIZER_SUITE)):
        if not isinstance(document[kind], dict):
            raise ValueError(f'{kind} must map names to suite lists')
        for owner, suites in document[kind].items():
            if not isinstance(suites, list) or not suites:
                raise ValueError(f'{kind}.{owner} must be a non-empty list')
            names = [suite.get('name') if isinstance(suite, dict) else None for suite in suites]
            if len(set(names)) != len(names):
                raise ValueError(f'{kind}.{owner}: suite names must be unique')
            for suite in suites:
                if not isinstance(suite, dict) or set(suite) != fields or not NAME.match(str(suite['name'])) \
                        or not SHA256.match(str(suite['report_sha256'])):
                    raise ValueError(f'{kind}.{owner}: each suite needs exactly {sorted(fields)}')
                if kind == 'skills' and canonical_sha256(suite['protocol']) != suite['protocol_sha256']:
                    raise ValueError(f'skills.{owner}.{suite["name"]}: protocol_sha256 does not match its protocol')
                if kind == 'recognizers' and (not SHA256.match(str(suite['manifest_sha256']))
                                              or suite['split'] != 'heldout' or not isinstance(suite['labels'], list)
                                              or not suite['labels']):
                    raise ValueError(f'recognizers.{owner}.{suite["name"]}: needs a held-out manifest and labels')
    return document


def save_suites(path, document):
    """Validated, then atomically replaced, keeping the file's leading comment block."""
    path = Path(path or default_path())
    header = ''.join(line for line in path.read_text().splitlines(keepends=True) if line.startswith('#')) \
        if path.is_file() else ''
    temporary = path.with_name(f'.{path.name}.tmp')
    temporary.write_text(header + yaml.safe_dump(document, sort_keys=True, default_flow_style=False))
    try:
        load_suites(temporary)
    except (OSError, ValueError):
        temporary.unlink()
        raise
    os.replace(temporary, path)


def store_report(report_path, store=None):
    """Keep a report content-addressed; returns its SHA-256."""
    data = Path(report_path).read_bytes()
    sha256 = hashlib.sha256(data).hexdigest()
    directory = Path(store or STORE)
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / f'{sha256}.json'
    if not target.is_file():
        temporary = directory / f'.{sha256}.tmp'
        shutil.copyfile(report_path, temporary)
        os.replace(temporary, target)
    return sha256


def stored_report(sha256, store=None):
    if not SHA256.match(str(sha256)):
        raise ValueError('a stored report is named by its SHA-256')
    path = Path(store or STORE) / f'{sha256}.json'
    if not path.is_file():
        raise ValueError(f'stored report {sha256[:12]} is missing from {path.parent}')
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != sha256:
        raise ValueError(f'stored report {sha256[:12]} does not match its digest')
    return json.loads(data)
