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


def test_optical_measurement_recovers_known_rigid_motion_and_rejects_deformation():
    import runpy
    from pathlib import Path
    import numpy as np
    functions = runpy.run_path(str(Path(__file__).parents[1]/'scripts/test-braking.py'))
    square = np.array([[0,0],[35,0],[35,35],[0,35]],dtype=float)
    markers = {53:square,69:square+[0,50],59:square-[0,50]}
    matrix,origin,reference,error = functions['metric_reference'](markers,[53,69,59],.035)
    assert error < 1e-8  # OpenCV's homography inputs use float32.
    rotation = np.array([[math.cos(.2),-math.sin(.2)],[math.sin(.2),math.cos(.2)]])
    moved = {key:(corners-origin)@rotation.T+origin+[10,5] for key,corners in markers.items()}
    pose,_,_ = functions['metric_pose'](moved,matrix,origin,reference)
    assert pose == pytest.approx([.01,.005,.2])
    assert functions['metric_pose']({53:markers[53]},matrix,origin,reference) is None
    moved[59] += [20,0]
    assert functions['metric_pose'](moved,matrix,origin,reference) is None


def test_marker_roi_preserves_full_image_coordinates_and_recovers_after_a_shift():
    import cv2
    import runpy
    from pathlib import Path
    import numpy as np
    Camera = runpy.run_path(str(Path(__file__).parents[1]/'scripts/test-braking.py'))['Camera']
    camera = Camera.__new__(Camera)
    camera.config = {'marker_ids':[53,69,59]}
    camera.roi = None
    camera.detector = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    camera.parameters = cv2.aruco.DetectorParameters_create()
    image = np.full((600,1000),255,dtype=np.uint8)
    for index,key in enumerate(camera.config['marker_ids']):
        image[100:180,100+100*index:180+100*index] = cv2.aruco.drawMarker(camera.detector,key,80)
    original = camera.detect(image)
    assert set(original)=={53,69,59} and camera.roi is not None
    cropped = camera.detect(image)
    assert all(np.allclose(original[key],cropped[key],atol=1) for key in original)
    shifted = np.full_like(image,255)
    shifted[300:500,500:900] = image[0:200,0:400]
    recovered = camera.detect(shifted)
    assert set(recovered)==set(original)
    assert all(np.allclose(recovered[key],original[key]+[500,300],atol=1) for key in original)


def test_marker_plane_rectification_recovers_motion_under_perspective():
    import cv2
    import runpy
    from pathlib import Path
    import numpy as np
    functions = runpy.run_path(str(Path(__file__).parents[1]/'scripts/test-braking.py'))
    square = np.array([[-.0175,-.0175],[.0175,-.0175],[.0175,.0175],[-.0175,.0175]])
    plane = {53:square,69:square+[.07,0],59:square+[-.07,0]}
    camera = np.array([[1800.,400,640],[120,1900,400],[1.5,3,1]])
    def project(corners):
        return cv2.perspectiveTransform(corners.reshape(-1,1,2),camera).reshape(-1,2)
    markers = {key:project(c) for key,c in plane.items()}
    matrix,origin,reference,error = functions['metric_reference'](markers,[53,69,59],.035)
    assert error<1e-6
    rotation = np.array([[math.cos(.12),-math.sin(.12)],[math.sin(.12),math.cos(.12)]])
    moved = {key:project(c@rotation.T+[.02,-.015]) for key,c in plane.items()}
    pose,_,_ = functions['metric_pose'](moved,matrix,origin,reference)
    assert pose==pytest.approx([.02,-.015,.12],abs=1e-6)


def test_stopping_clearance_uses_farthest_point_excursion_without_summing_jitter():
    import runpy
    from pathlib import Path
    functions = runpy.run_path(str(Path(__file__).parents[1]/'scripts/test-braking.py'))
    excursion = functions['maximum_swept_excursion']
    assert excursion([[0,0,0],*[p for _ in range(100) for p in [[.001,0,0],[0,0,0]]]],.53) == .001
    assert excursion([[0,0,0],[.01,0,.2],[0,0,0]],.53) == pytest.approx(.01+1.06*math.sin(.1))


def test_optical_speed_uses_capture_intervals_and_rejects_replayed_timestamps():
    import runpy
    from pathlib import Path
    speeds = runpy.run_path(str(Path(__file__).parents[1]/'scripts/test-braking.py'))['observed_speeds']
    frames = [{'pts_ns':100_000_000,'time':1.,'pose':[0,0,0]},
              {'pts_ns':200_000_000,'time':2.,'pose':[.01,0,.02]}]
    assert speeds(frames,False) == pytest.approx([.1])
    assert speeds(frames,True) == pytest.approx([.2])
    frames[1]['pts_ns'] = frames[0]['pts_ns']
    with pytest.raises(RuntimeError,match='capture timestamps'):
        speeds(frames,False)


def test_braking_observes_terminal_speed_despite_stationary_startup(tmp_path,monkeypatch):
    import runpy
    import time
    from pathlib import Path
    BrakingTest = runpy.run_path(str(Path(__file__).parents[1]/'scripts/test-braking.py'))['BrakingTest']
    stamp = time.monotonic()
    frames = [{'time':stamp+i*.04,'pts_ns':int((i+1)*4e7),'pose':[i*.002,0,0]} for i in range(10)]
    def tick(command):
        node.optical_samples.extend(frames)
        node.camera.last_pose = [.03,0,0]
    node = types.SimpleNamespace(center=(0,0,0),camera=types.SimpleNamespace(last_pose=[0,0,0]),
        flags={'base_motion_permitted':True},motion_pauses=0,paused_seconds=0,health_faults=[],
        move=lambda target:None,wait=lambda *args:None,tick=tick,optical_samples=[],
        command=types.SimpleNamespace(publish=lambda command:None),settle=lambda:frames,
        config={'trial_displacement_m':.03,'measurement_uncertainty_m':.01,
                'marker_center_offset_bound_m':.20,'maximum_stopping_distance_m':.05,'maximum_stop_time_s':1.5},
        checks={'trials':[]},output=tmp_path)
    monkeypatch.setitem(BrakingTest.trial.__globals__,'observed_speeds',lambda *args:[0]*20+[.05]*3)
    result = BrakingTest.trial(node,'forward',.05)
    assert result['median_observed_speed']==0 and result['requested_speed_covered']
    monkeypatch.setitem(BrakingTest.trial.__globals__,'observed_speeds',lambda *args:[0]*23)
    node.camera.last_pose = [0,0,0]
    with pytest.raises(RuntimeError,match='not independently observed'):
        BrakingTest.trial(node,'forward',.05)


def test_optical_return_handles_reflected_axes_and_caps_both_commands():
    import runpy
    from pathlib import Path
    import numpy as np
    command = runpy.run_path(str(Path(__file__).parents[1]/'scripts/test-braking.py'))['optical_return_twist']
    straight = command([.01,0,0],[0,0,0],np.eye(2))
    assert straight.linear.x == pytest.approx(-.02)
    reflected = command([0,1,.5],[0,0,0],np.diag([1,-1]))
    assert math.hypot(reflected.linear.x,reflected.linear.y)==pytest.approx(.05)
    assert reflected.linear.y>0 and reflected.angular.z==.20
    assert command([0,0,0],[0,0,0],np.eye(2)).linear.x==0


@pytest.mark.parametrize('visible',[False,True])
def test_missing_measurement_camera_does_not_interrupt_production(monkeypatch,tmp_path,visible):
    import runpy
    import sys
    from pathlib import Path
    root = Path(__file__).parents[1]
    functions = runpy.run_path(str(root/'scripts/test-braking.py'))
    globals_ = functions['main'].__globals__
    (tmp_path/'config').mkdir()
    (tmp_path/'config/physical_test.yaml').write_text((root/'config/physical_test.yaml').read_text())
    monkeypatch.setitem(globals_,'ROOT',tmp_path)
    monkeypatch.setattr(sys,'argv',['test-braking.py'])
    def missing_camera(*args):
        raise RuntimeError('measurement camera missing')
    if visible:
        camera = types.SimpleNamespace(calibration=None,sample=lambda:None,close=lambda:None)
        monkeypatch.setitem(globals_,'Camera',lambda *args:camera)
        clock = iter([0,11])
        monkeypatch.setattr(globals_['time'],'monotonic',lambda:next(clock))
    else:
        monkeypatch.setitem(globals_,'Camera',missing_camera)
    calls = []
    monkeypatch.setitem(globals_['navigation'],'main',lambda *args:calls.append(args))
    with pytest.raises(RuntimeError,match='measurement camera'):
        functions['main']()
    assert not calls


def test_braking_exploration_stops_a_speed_step_at_its_first_failed_direction(tmp_path):
    import runpy
    from pathlib import Path
    BrakingTest = runpy.run_path(str(Path(__file__).parents[1]/'scripts/test-braking.py'))['BrakingTest']
    trials = []
    def trial(direction,speed):
        trials.append((direction,speed))
        return {'within_budget':direction=='forward','feedback_interrupted':False}
    def finished(*args):
        raise RuntimeError('exploration finished')
    node = types.SimpleNamespace(camera=types.SimpleNamespace(calibration=[0,0,0,0],last_pose=[0,0,0]),
        pose=(0,0,0),flags={'base_motion_permitted':True},checks={},output=tmp_path,
        config={'linear_steps_m_s':[.03,.05],'angular_steps_rad_s':[],'trials_per_direction':0},
        wait=lambda *args:None,wait_ready=lambda:None,settle=lambda:None,calibrate_return=lambda:None,stop=lambda:None,
        trial=trial,move=lambda *args:None,buffer=types.SimpleNamespace(lookup_transform=finished))
    with pytest.raises(RuntimeError,match='exploration finished'):
        BrakingTest.run(node)
    assert trials == [('forward',.03),('reverse',.03)]
    trials.clear()
    node.config['directions'] = ['left']
    with pytest.raises(RuntimeError,match='exploration finished'):
        BrakingTest.run(node)
    assert trials == [('left',.03)]


def test_indoor_stop_zone_and_speed_profile_match_the_acceptance_budget():
    from pathlib import Path
    import yaml
    from lekiwi_rmf.safety_supervisor import _polygon, _polygon_boundary_distance, _point_in_polygon
    root = Path(__file__).parents[1]
    nav2 = yaml.safe_load((root/'config/nav2_params.yaml').read_text())
    profile = yaml.safe_load((root/'config/physical_test.yaml').read_text())
    footprint = _polygon(nav2['local_costmap']['local_costmap']['ros__parameters']['footprint'],'footprint')
    stop = _polygon(nav2['collision_monitor']['ros__parameters']['StopZone']['points'],'stop')
    assert all(_point_in_polygon(point,stop) for point in footprint)
    assert _polygon_boundary_distance(footprint,stop) == pytest.approx(profile['maximum_stopping_distance_m'])
    assert load_base_speed_limits(root/'config/nav2_params.yaml') == (
        profile['maximum_linear_speed_m_s'],profile['maximum_angular_speed_rad_s'])
    assert nav2['velocity_smoother']['ros__parameters']['max_velocity'] == [
        profile['maximum_linear_speed_m_s'],profile['maximum_linear_speed_m_s'],
        profile['maximum_angular_speed_rad_s']]


def test_runner_drains_callbacks_and_reports_a_physical_stop(monkeypatch):
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
    with pytest.raises(RuntimeError,match='physical obstacle'):
        functions['Test'].tick(node,command)
    assert len(calls) == 21
    assert commands[-1].linear.x == commands[-1].angular.z == 0


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
        motion_pauses=0,paused_seconds=0.0,
        lease=types.SimpleNamespace(publish=lambda m:leases.append(m.data)),
        command=types.SimpleNamespace(publish=commands.append))
    node.tick = lambda *args,**kwargs:Test.tick(node,*args,**kwargs)
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
