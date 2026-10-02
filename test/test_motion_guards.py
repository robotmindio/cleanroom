"""Shared lease, Twist, duration and stamp checks used by the motion gates."""

import math
import types

import pytest

from lekiwi_rmf.motion_guards import (
    lease_is_fresh, load_base_speed_limits, positive_seconds_ns, stamp_ns, twist_is_finite,
)


def test_base_speed_limits_reject_bad_configuration_and_include_reverse(tmp_path):
    import yaml
    path = tmp_path / "nav2.yaml"
    controller = {"vx_max": 0.2, "vx_min": -0.3, "vy_max": 0.1, "wz_max": 0.5}
    def write():
        path.write_text(yaml.safe_dump({"controller_server": {
            "ros__parameters": {"FollowPath": controller},
        }}))
    write()
    assert load_base_speed_limits(path) == (0.3, 0.5)
    for bad in (True, "0.3", 0, -0.1, float("inf"), float("nan")):
        controller["vx_max"] = bad
        write()
        with pytest.raises(ValueError, match="invalid Nav2 speed limits"):
            load_base_speed_limits(path)
    path.write_text("[]")
    with pytest.raises(ValueError, match="invalid Nav2 speed limits"):
        load_base_speed_limits(path)


def test_lease_is_fresh_only_within_its_timeout_and_never_from_the_future():
    assert lease_is_fresh(1_000, 100, 1_100)
    assert not lease_is_fresh(1_000, 100, 1_101)
    assert not lease_is_fresh(1_000, 100, 999)
    assert not lease_is_fresh(None, 100, 1_000)


def test_every_twist_axis_must_be_finite():
    def twist(z=0.0):
        vector = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        return types.SimpleNamespace(linear=vector, angular=types.SimpleNamespace(x=0.0, y=0.0, z=z))

    assert twist_is_finite(twist())
    assert not twist_is_finite(twist(z=math.nan))


@pytest.mark.parametrize("value", [0.0, -1.0, math.inf, math.nan])
def test_durations_must_be_finite_and_positive(value):
    with pytest.raises(ValueError, match="timeout must be finite and positive"):
        positive_seconds_ns(value, "timeout")


def test_durations_and_stamps_convert_to_nanoseconds():
    assert positive_seconds_ns(0.25, "timeout") == 250_000_000
    assert stamp_ns(types.SimpleNamespace(sec=2, nanosec=5)) == 2_000_000_005


def test_base_test_stops_inside_authorized_radius_and_rejects_bad_odometry():
    from lekiwi_rmf.motion_guards import inside_base_test_boundary
    assert inside_base_test_boundary((1.19, 2.0, 0.0), (1.0, 2.0))
    assert not inside_base_test_boundary((1.0, 2.21, 0.0), (1.0, 2.0))
    assert not inside_base_test_boundary((math.nan, 2.0, 0.0), (1.0, 2.0))


def test_physical_fault_probe_restores_service_when_injection_fails():
    import runpy
    import time
    from pathlib import Path
    FaultTest = runpy.run_path(str(Path(__file__).parents[1] /
                                  'scripts/test-physical-acceptance.py'))['FaultTest']
    calls = []
    node = types.SimpleNamespace(flags={'base_motion_permitted':True,'arm_stowed':True},
        speed=0.0, odom_at=time.monotonic(), pose=(0,0,0),
        checks={},linear_speed=0.0,safe_speed=0.01,
        stop=lambda:calls.append('stop'), wait=lambda condition,timeout:condition())
    def tick(command):
        node.linear_speed = 0.01
        node.pose = (0.005,0,0)
    node.tick = tick
    def broken(command):
        raise RuntimeError('injection failed')
    with pytest.raises(RuntimeError,match='injection failed'):
        FaultTest.fault(node,'scan_disconnect',broken,lambda:calls.append('restore'),'scan:')
    assert calls==['stop','restore']


def test_physical_probe_limits_lifecycle_polling_to_two_queries_per_second(monkeypatch):
    import runpy
    from pathlib import Path
    FaultTest = runpy.run_path(str(Path(__file__).parents[1] /
                                  'scripts/test-physical-acceptance.py'))['FaultTest']
    now, queries = [0.0], []
    def request(_):
        queries.append(now[0])
        state = 3 if len(queries)==3 else 1
        return types.SimpleNamespace(done=lambda:True,
            result=lambda:types.SimpleNamespace(current_state=types.SimpleNamespace(id=state)))
    node = types.SimpleNamespace(pose=(0,0,0),restart_tests=True,
        flags={'arm_stowed':True,'driver':'ARMED'},
        monitor_state=types.SimpleNamespace(service_is_ready=lambda:True,call_async=request),
        wait=lambda condition,timeout:condition(),
        tick=lambda command:now.__setitem__(0,now[0]+0.05),fault=lambda *args:None)
    monkeypatch.setitem(FaultTest.run.__globals__,'time',types.SimpleNamespace(monotonic=lambda:now[0]))
    monkeypatch.setitem(FaultTest.run.__globals__,'subprocess',
                        types.SimpleNamespace(check_output=lambda *args,**kwargs:'123'))
    FaultTest.run(node)
    assert len(queries)==3 and all(b-a>=0.49 for a,b in zip(queries,queries[1:]))


def test_navigation_probe_withdraws_lease_until_wheel_feedback_recovers(monkeypatch):
    import runpy
    import time
    from pathlib import Path
    import rclpy

    Test = runpy.run_path(str(Path(__file__).parents[1] / 'scripts/test-navigation.py'))['Test']
    leases, commands = [], []
    node = types.SimpleNamespace(active=True, center=(0,0,0), pose=(0,0,0),
        odom_at=time.monotonic()-1, deadline=time.monotonic()+10,
        flags={'arm_stowed':True,'driver':'ARMED'},
        lease=types.SimpleNamespace(publish=lambda m:leases.append(m.data)),
        command=types.SimpleNamespace(publish=commands.append))
    node.tick = lambda *args,**kwargs:Test.tick(node,*args,**kwargs)
    def spin_once(node,timeout_sec):
        if not node.active:
            node.odom_at=time.monotonic()
    monkeypatch.setattr(rclpy,'spin_once',spin_once)
    node.tick()
    assert leases == [True,False] and node.active
    assert len(commands)==1 and commands[0].linear.x==commands[0].angular.z==0
