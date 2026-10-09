"""Shared lease, Twist, duration and stamp checks used by the motion gates."""

import math
import types

import pytest

from lekiwi_rmf.motion_guards import (
    bounded_test_speed_limits, lease_is_fresh, load_base_speed_limits, positive_seconds_ns, stamp_ns, twist_is_finite,
)


def test_attended_speed_trials_cannot_exceed_production_limits():
    assert bounded_test_speed_limits(.3, 1.57, (.3, 1.57)) == (.3, 1.57)
    for values in [(True,.2), (math.nan,.2), (.03,0), (.31,.2), (.03,1.58)]:
        with pytest.raises(ValueError):
            bounded_test_speed_limits(*values, (.3,1.57))


def test_qualification_profiles_fit_the_one_metre_fixture_and_reject_missing_reserves(tmp_path):
    from pathlib import Path
    import yaml
    from lekiwi_rmf.motion_guards import load_base_test_profile
    root=Path(__file__).parents[1]/'config'
    for stage in ('0.20','0.25','0.30'):
        profile=load_base_test_profile(root/'nav2_params.yaml',stage)
        assert profile['linear_speed_m_s']==float(stage)
        assert profile['required_clearance_m']==1.
        assert profile['nominal_command_duration_s']==1.6
        assert profile['measurement_uncertainty_m']==.04
        assert profile['point_speed_bound_m_s']==pytest.approx(float(stage)*1.15)
        assert profile['translation_stopping_margin_m']==pytest.approx(float(stage)*1.15*1.5+.04)
        assert profile['return_stopping_margin_m']==pytest.approx((.10+.33*.06)*1.15*1.5+.04)
    for stage in ('0.06','0.10','0.40',''):
        with pytest.raises(ValueError):
            load_base_test_profile(root/'nav2_params.yaml',stage)
    (tmp_path/'onboard_braking.yaml').write_text((root/'onboard_braking.yaml').read_text())
    data=yaml.safe_load((root/'base_speed_qualification.yaml').read_text())
    for key,value in [('required_clearance_m',.8),('driver_center_radius_m',.99),
                      ('nominal_command_duration_s',10.),('linear_speed_m_s',True),
                      ('angular_speed_rad_s',2.),('return_angular_speed_rad_s',.6),
                      ('measurement_uncertainty_m',.05),('measurement_uncertainty_m',True)]:
        changed=yaml.safe_load(yaml.safe_dump(data))
        destination=changed['stages']['0.30'] if key.endswith('speed_m_s') or key=='angular_speed_rad_s' else changed['bounds']
        destination[key]=value
        (tmp_path/'base_speed_qualification.yaml').write_text(yaml.safe_dump(changed))
        with pytest.raises(ValueError):
            load_base_test_profile(tmp_path/'nav2_params.yaml','0.30')


def test_indoor_stop_zone_and_speed_profile_match_the_acceptance_budget():
    from pathlib import Path
    import yaml
    from lekiwi_rmf.geometry import (
        point_in_polygon as _point_in_polygon, polygon as _polygon,
        polygon_boundary_distance as _polygon_boundary_distance)
    root = Path(__file__).parents[1]
    nav2 = yaml.safe_load((root/'config/nav2_params.yaml').read_text())
    profile = yaml.safe_load((root/'config/onboard_braking.yaml').read_text())
    footprint = _polygon(nav2['local_costmap']['local_costmap']['ros__parameters']['footprint'],'footprint')
    stop = _polygon(nav2['collision_monitor']['ros__parameters']['StopZone']['points'],'stop')
    assert all(_point_in_polygon(point,stop) for point in footprint)
    assert _polygon_boundary_distance(footprint,stop) == pytest.approx(profile['maximum_stopping_distance_m'])
    linear, angular = load_base_speed_limits(root/'config/nav2_params.yaml')
    assert nav2['velocity_smoother']['ros__parameters']['max_velocity'] == [linear, linear, angular]


@pytest.mark.parametrize('feedback_check',[True,False])
def test_runner_drains_callbacks_and_reports_a_physical_stop(monkeypatch,feedback_check):
    import runpy
    import time
    from pathlib import Path
    from geometry_msgs.msg import Twist
    functions = runpy.run_path(str(Path(__file__).parents[1]/'scripts/test-navigation.py'))
    calls, commands = [], []
    monkeypatch.setattr(functions['rclpy'],'spin_once',lambda *a,**k:calls.append(k))
    node = types.SimpleNamespace(active=True,lease=types.SimpleNamespace(publish=lambda m:None),
                                 command=types.SimpleNamespace(publish=commands.append),
                                 monitor_action=('StopZone',1),blocked_at=time.monotonic()-4)
    command = Twist()
    command.linear.x = .03
    with pytest.raises(RuntimeError,match='StopZone blocks motion'):
        functions['Test'].tick(node,command,check=feedback_check)
    assert len(calls) == 21
    assert commands[-1].linear.x == commands[-1].angular.z == 0
    node.phase='scan_disconnect'
    node.center=None
    functions['Test'].tick(node,command,check=False)
    assert commands[-1].linear.x==.03


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


def test_shared_runner_uses_current_production_caps_and_restores_on_launch_failure(tmp_path,monkeypatch):
    import runpy
    from pathlib import Path
    main = runpy.run_path(str(Path(__file__).parents[1]/'scripts/test-navigation.py'))['main']
    calls=[]
    def run(argv,**kwargs):
        calls.append(argv)
        return types.SimpleNamespace(returncode=0)
    def launch(argv,**kwargs):
        calls.append(argv)
        raise RuntimeError('launch prepared')
    monkeypatch.setitem(main.__globals__,'subprocess',types.SimpleNamespace(run=run,Popen=launch,STDOUT=-2))
    monkeypatch.setitem(main.__globals__,'installed_stack_arguments',lambda:['remote_ip:=127.0.0.1'])
    with pytest.raises(RuntimeError,match='launch prepared'):
        main(output=tmp_path)
    linear,angular=load_base_speed_limits(Path(__file__).parents[1]/'config/nav2_params.yaml')
    assert f'base_test_linear_limit:={linear}' in calls[2]
    assert f'base_test_angular_limit:={angular}' in calls[2]
    assert calls[-1][-2:]==['start','lekiwi-stack.service']


def test_shared_runner_restores_production_if_the_test_stack_has_already_exited(tmp_path,monkeypatch):
    import runpy
    from pathlib import Path
    main=runpy.run_path(str(Path(__file__).parents[1]/'scripts/test-navigation.py'))['main']
    calls=[]
    monkeypatch.setitem(main.__globals__,'installed_stack_arguments',lambda:['remote_ip:=127.0.0.1'])
    monkeypatch.setitem(main.__globals__,'subprocess',types.SimpleNamespace(STDOUT=-2,
        run=lambda argv,**kwargs:(calls.append(argv) or types.SimpleNamespace(returncode=0)),
        Popen=lambda *args,**kwargs:types.SimpleNamespace(poll=lambda:0)))
    monkeypatch.setitem(main.__globals__,'rclpy',types.SimpleNamespace(init=lambda **kwargs:None))
    def broken():
        raise RuntimeError('test construction failed')
    with pytest.raises(RuntimeError,match='test construction failed'):
        main(broken,tmp_path)
    assert calls[-1][-2:]==['start','lekiwi-stack.service']


@pytest.mark.parametrize('angular_test',[False,True])
def test_physical_fault_probe_restores_service_when_injection_fails(angular_test):
    import runpy
    import time
    from pathlib import Path
    FaultTest = runpy.run_path(str(Path(__file__).parents[1] /
                                  'scripts/test-physical-acceptance.py'))['FaultTest']
    calls = []
    node = types.SimpleNamespace(flags={'base_motion_permitted':True,'arm_stowed':True},
        speed=0.0, odom_at=time.monotonic(), pose=(0,0,0),
        checks={},measured_speed=0.0,safe_speed=0.01,test_speed=.05,monitor_action=None,angular_test=angular_test,
        stop=lambda:calls.append('stop'), wait=lambda condition,timeout:condition())
    def tick(command):
        assert command.angular.z==(.05 if angular_test else 0)
        assert command.linear.x==(0 if angular_test else .05)
        calls.append('tick')
        node.measured_speed = .025 if calls.count('tick')==1 else .05
        node.safe_speed = .05
        node.pose = (0.005,0,.02 if angular_test else 0)
    node.tick = tick
    def broken(command):
        raise RuntimeError('injection failed')
    with pytest.raises(RuntimeError,match='injection failed'):
        FaultTest.fault(node,'scan_disconnect',broken,lambda:calls.append('restore'),'scan:')
    assert calls==['tick','tick','stop','restore']
    assert node.checks['scan_disconnect']['injection_speed']==.05


def test_fault_probe_stops_before_injection_if_collision_monitor_reduces_speed():
    import runpy
    import time
    from pathlib import Path
    FaultTest=runpy.run_path(str(Path(__file__).parents[1]/'scripts/test-physical-acceptance.py'))['FaultTest']
    calls=[]
    node=types.SimpleNamespace(flags={'base_motion_permitted':True,'arm_stowed':True},
        measured_speed=.0175,safe_speed=.0175,test_speed=.05,pose=(0,0,0),angular_test=False,
        odom_at=time.monotonic(),monitor_action=('SlowdownZone',2),
        stop=lambda:calls.append('stop'),wait=lambda condition,timeout:condition())
    with pytest.raises(RuntimeError,match='blocked by collision monitor'):
        FaultTest.fault(node,'scan_disconnect',lambda c:calls.append('inject'),lambda:None,'scan:')
    assert calls==['stop']


def test_shared_motion_probe_waits_for_active_monitor_without_nonzero_commands(monkeypatch):
    import runpy
    from pathlib import Path
    Test = runpy.run_path(str(Path(__file__).parents[1] /
                             'scripts/test-navigation.py'))['Test']
    now, queries, commands = [0.0], [], []
    def request(_):
        queries.append(now[0])
        state = 3 if len(queries)==3 else 1
        return types.SimpleNamespace(done=lambda:True,
            result=lambda:types.SimpleNamespace(current_state=types.SimpleNamespace(id=state)))
    def tick(command):
        commands.append(command)
        now[0] += .05
    node = types.SimpleNamespace(pose=(0,0,0),active=False,
        flags={'arm_stowed':True,'driver':'ARMED'},
        monitor_state=types.SimpleNamespace(service_is_ready=lambda:True,call_async=request),
        wait=lambda condition,timeout:condition(),
        tick=tick)
    monkeypatch.setitem(Test.wait_ready.__globals__,'time',types.SimpleNamespace(monotonic=lambda:now[0]))
    Test.wait_ready(node)
    assert len(queries)==3 and all(b-a>=0.49 for a,b in zip(queries,queries[1:]))
    assert not node.active and commands
    assert all(c.linear.x==c.linear.y==c.angular.z==0 for c in commands)


@pytest.mark.parametrize('unverified',['wheel','arm'])
def test_navigation_probe_withdraws_lease_until_feedback_recovers(monkeypatch,unverified):
    import runpy
    import time
    from pathlib import Path
    import rclpy

    Test = runpy.run_path(str(Path(__file__).parents[1] / 'scripts/test-navigation.py'))['Test']
    leases, commands = [], []
    node = types.SimpleNamespace(active=True, center=(0,0,0), pose=(0,0,0),
        odom_at=time.monotonic()-(1 if unverified=='wheel' else 0), deadline=time.monotonic()+10,
        flags={'arm_stowed':unverified!='arm','driver':'ARMED'},health={},
        motion_pauses=0,
        lease=types.SimpleNamespace(publish=lambda m:leases.append(m.data)),
        command=types.SimpleNamespace(publish=commands.append))
    node.tick = lambda *args,**kwargs:Test.tick(node,*args,**kwargs)
    node.check_feedback = lambda:Test.check_feedback(node)
    node.pause_until = lambda *args:Test.pause_until(node,*args)
    def spin_once(node,timeout_sec):
        if not node.active:
            node.odom_at=time.monotonic()
            node.flags['arm_stowed']=True
    monkeypatch.setattr(rclpy,'spin_once',spin_once)
    node.tick()
    assert leases == [True,False] and node.active
    assert len(commands)==1 and commands[0].linear.x==commands[0].angular.z==0
    assert node.motion_pauses==1
