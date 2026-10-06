"""The onboard observer rejects bad/stale poses before another motion tick."""
import runpy
from pathlib import Path
from types import SimpleNamespace
import time
import json

import pytest


@pytest.mark.parametrize('cleanup_failure',[False,True])
def test_production_runner_records_payload_without_restarting_or_shutting_down(monkeypatch, tmp_path, cleanup_failure):
    module = runpy.run_path(str(Path(__file__).parents[1] / 'scripts/test-navigation.py'))
    main = module['main']
    shared = main.__globals__
    commands = []
    released = []
    monkeypatch.setitem(shared, 'installed_stack_arguments', lambda: [])
    def system_command(args, **kwargs):
        commands.append(args)
        assert args == ['systemctl', 'is-active', '--quiet', 'lekiwi-stack.service']
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(shared['subprocess'], 'run', system_command)
    monkeypatch.setattr(shared['subprocess'], 'check_output', lambda *args, **kwargs: 'verified-source\n')
    monkeypatch.setattr(shared['rclpy'], 'init', lambda **kwargs: None)
    monkeypatch.setattr(shared['rclpy'], 'try_shutdown', lambda: released.append('ros'))
    def forbidden(*args, **kwargs):
        raise AssertionError('production services must remain running')
    monkeypatch.setattr(shared['subprocess'], 'Popen', forbidden)
    node = SimpleNamespace(
        goal=None, center=(0., 0., 0.), pose=(0., 0., 0.), counts={}, health={},
        trace=[(0., 0., 0.)], slam=[], health_faults=[], monitor_action=None,
        map_client=SimpleNamespace(wait_for_service=lambda **kwargs: False),
        lifecycle_client=SimpleNamespace(wait_for_service=forbidden),
        listener=SimpleNamespace(unregister=lambda: released.append('listener')),
        navigation=SimpleNamespace(destroy=lambda: released.append('navigation')),
        run=lambda: None, stop=lambda: None, destroy_node=lambda: released.append('node'),
    )
    if cleanup_failure:
        def fail_stop():raise RuntimeError('callback failed while stopping')
        node.stop=fail_stop
        with pytest.raises(RuntimeError,match='callback failed'):
            main(lambda:node,tmp_path,production=True,payload_kg=.2)
    else:
        main(lambda: node, tmp_path, production=True, payload_kg=.2)
        result = json.loads((tmp_path / 'result.json').read_text())
        assert result['reported_payload_kg'] == .2 and result['mode'] == 'production'
    assert released==['listener','navigation','node','ros']
    assert len(commands) == 1
    for mass in (-1., float('nan'), float('inf')):
        with pytest.raises(ValueError):
            main(lambda: node, tmp_path, production=True, payload_kg=mass)


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
