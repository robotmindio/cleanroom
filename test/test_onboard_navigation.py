"""The onboard observer rejects bad/stale poses before another motion tick."""
import runpy
from pathlib import Path
from types import SimpleNamespace

import pytest


def test_onboard_visual_boundary_and_capture_age(monkeypatch):
    module = runpy.run_path(str(Path(__file__).parents[1] /
                               'scripts/test-onboard-navigation.py'))
    check = module['OnboardTest'].check_visual_feedback
    sample = {'t': 10., 'source_age_s': .4, 'x': .08, 'y': .02, 'yaw': .1}
    info = [{'lost': False}]
    check(SimpleNamespace(visual=[sample], visual_info=info), 10.5)
    for change in ({'source_age_s': 2.1}, {'source_age_s': -.3},
                   {'x': .17}, {'x': float('nan')}):
        with pytest.raises(RuntimeError):
            check(SimpleNamespace(visual=[{**sample, **change}], visual_info=info), 10.)
    with pytest.raises(RuntimeError):
        check(SimpleNamespace(visual=[]), 10.)
    for unavailable in ([], [{'lost': True}]):
        with pytest.raises(RuntimeError):
            check(SimpleNamespace(visual=[sample], visual_info=unavailable), 10.)
    published = []
    monkeypatch.setattr(module['NAV']['Test'], 'tick', lambda *a: published.append(True))
    node = object.__new__(module['OnboardTest'])
    node.center, node.visual = (0., 0., 0.), []
    with pytest.raises(RuntimeError):
        node.tick(object())
    assert not published
