"""The onboard observer rejects bad/stale poses before another motion tick."""
import runpy
from pathlib import Path
from types import SimpleNamespace
import time

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
    node = object.__new__(module['OnboardTest'])
    node.center, node.pose, node.visual, node.visual_info = (0.,0.,0.), (0.,0.,0.), [], []
    node.active, node.flags, node.health = True, {'driver':'ARMED','arm_stowed':True}, {}
    node.odom_at, node.deadline = time.monotonic(), time.monotonic()+20
    node.motion_pauses, node.paused_seconds, node.monitor_action = 0, 0., None
    node.lease = SimpleNamespace(publish=lambda m:None)
    node.command = SimpleNamespace(publish=published.append)
    def receive(*a, **k):
        node.visual = [{**sample, 't':time.monotonic()}]
        node.visual_info = info
    monkeypatch.setattr(module['NAV']['rclpy'], 'spin_once', receive)
    command = module['NAV']['Twist']()
    command.linear.x = .03
    node.tick(command)  # Queued feedback must be drained before rejecting it.
    assert published == [command]
    node.visual_info = [{'lost':True}]
    def recover(*a, **k):
        if not node.active:
            receive()
    monkeypatch.setattr(module['NAV']['rclpy'], 'spin_once', recover)
    node.tick(command)
    assert published[-2].linear.x == 0 and published[-1] is command
    assert node.motion_pauses == 1 and node.active
    monkeypatch.setattr(module['NAV']['rclpy'], 'spin_once', lambda *a,**k:None)
    node.visual[-1]['x'] = .17
    count = len(published)
    with pytest.raises(RuntimeError, match='16 cm'):
        node.tick(command)
    assert len(published) == count
