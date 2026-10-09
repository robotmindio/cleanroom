from importlib import import_module

import pytest
import numpy as np


def test_short_qualification_pulses_are_centered_in_the_fixture():
    from types import SimpleNamespace
    pulse=import_module('test-onboard-braking').OnboardBraking.pulse_start
    node=SimpleNamespace(center=(0.,0.,np.pi/2),config={'stage':'0.30',
        'nominal_command_duration_s':1.2,'linear_speed_m_s':.3,'angular_speed_rad_s':.6})
    assert pulse(node,'forward')==pytest.approx((0.,-.18,np.pi/2))
    assert pulse(node,'reverse')==pytest.approx((0.,.18,np.pi/2))
    assert pulse(node,'left')==pytest.approx((.18,0.,np.pi/2))
    assert pulse(node,'rotation_ccw')==pytest.approx((0.,0.,np.pi/2-.36))


def test_velocity_seed_does_not_replace_the_independent_wall_measurement():
    from types import SimpleNamespace
    import time
    from sensor_msgs.msg import LaserScan
    module=import_module('test-onboard-braking')
    angles=np.arange(720)*2*np.pi/720-np.pi
    def wall_scan(position,stamp):
        directions=np.column_stack((np.cos(angles),np.sin(angles)))
        distances=np.minimum((2-np.sign(directions[:,0])*position)/abs(directions[:,0]),
                             2/np.maximum(abs(directions[:,1]),1e-12))
        scan=LaserScan(angle_min=-np.pi,angle_increment=2*np.pi/720,range_min=.05,range_max=10.,ranges=distances.astype(float).tolist())
        scan.header.frame_id='lidar'
        scan.header.stamp.nanosec=int(stamp*1e9)
        return scan,distances[:,None]*directions
    _,reference=wall_scan(0.,0.)
    transform=SimpleNamespace(rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.),
                              translation=SimpleNamespace(x=0.,y=0.))
    for actual,stamp,seed in [(.04,.2,None),(0.,.5,None),(.15,.2,[.15,0.,0.])]:
        scan,_=wall_scan(actual,stamp)
        node=SimpleNamespace(sectors=[],scans=[],reference_points=reference,
            ranges=[] if seed else [{'pose':[0.,0.,0.],'stamp':0.}],wheel_velocity=(.3,0.,0.),odom_at=time.monotonic(),
            config={'body_radius_m':.33,**({'reference_start_pose':seed} if seed else {})},
            buffer=SimpleNamespace(can_transform=lambda *args:True,
                lookup_transform=lambda *args:SimpleNamespace(transform=transform)),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(stamp*1e9))))
        module.OnboardBraking.direct_scan(node,scan)
        assert node.ranges[-1]['pose']==pytest.approx([actual,0.,0.],abs=.001)


def test_resume_rejects_changed_speed_or_stopping_conditions():
    resume=import_module('test-onboard-braking').resumable_evidence
    config={'linear_speed_m_s':.03,'angular_speed_rad_s':.06,'payload_kg':.2,
            'measurement_uncertainty_m':.02,'maximum_stopping_distance_m':.05,
            'direction':'forward','depth_filter_pid':'123'}
    previous={'profile':{**config,'direction':None,'depth_filter_pid':'456'},
              'source_run':'earlier','trials':[{'qualification_eligible':True},
                                            {'qualification_eligible':False}],
              'faults':{'linear/scan_disconnect':{'passed':True,'independent':{'within_budget':True}},
                        'angular/scan_disconnect':{'passed':False}}}
    trials,faults=resume(previous,config)
    assert trials==[{'qualification_eligible':True,'source_run':'earlier'}]
    assert list(faults)==['linear/scan_disconnect']
    assert resume(previous,{**config,'trials_per_direction':1})==(trials,faults)
    assert resume(previous,{**config,'reference_start_pose':[.15,0.,0.]})==(trials,faults)
    for key in ('linear_speed_m_s','angular_speed_rad_s','payload_kg',
                'measurement_uncertainty_m','maximum_stopping_distance_m'):
        with pytest.raises(ValueError,match='profile differs'):
            resume(previous,{**config,key:config[key]*2})


@pytest.mark.parametrize('stage',[None,'0.30'])
def test_selected_stop_does_not_retry_and_qualification_returns_to_fixed_center(stage):
    from types import SimpleNamespace
    run=import_module('test-onboard-braking').OnboardBraking.run
    calls=[]
    node=SimpleNamespace(config={'direction':'forward','body_radius_m':.33,'test_center':(0.,0.,0.),'stage':stage},
        pose=(0.,0.,0.),ranges=[{'pose':[0.,0.,0.]}]*21,range_info=[{}],
        camera_poses=[{}],views={'front':1,'wrist':1,'astra':1},
        wait_ready=lambda:None,wait=lambda condition,timeout:condition(),save=lambda:None)
    def move(target,**limits):
        assert target==(0.,0.,0.)
        assert limits=={'linear_limit':.02,'angular_limit':.06} or (stage and limits=={})
        calls.append('center')
        node.ranges.append({'pose':[-.06,0.,0.]})
    def trial(direction):
        assert node.origin_range==([0.,0.,0.] if stage else [-.06,0.,0.])
        calls.append(direction)
        return True
    node.move,node.trial=move,trial
    run(node)
    completed=['center','forward']+(['center'] if stage else [])
    assert calls==completed
    node.trial=lambda direction:calls.append(direction) or False
    with pytest.raises(RuntimeError,match='unqualified'):
        run(node)
    assert calls==completed+['center','forward']


@pytest.mark.parametrize('qualified',[True,False])
def test_attended_sequence_stops_on_the_first_unqualified_trial(qualified):
    from types import SimpleNamespace
    module=import_module('test-onboard-braking')
    calls=[]
    node=SimpleNamespace(config={'stage':'0.20','body_radius_m':.33,
        'trials_per_direction':1,'nominal_only':True},pose=(0.,0.,0.),
        ranges=[{'pose':[0.,0.,0.]}]*21,range_info=[{}],camera_poses=[{}],
        views={'front':1,'wrist':1,'astra':1},trials=[],
        wait_ready=lambda:None,wait=lambda condition,timeout:condition(),move=lambda *a,**k:None)
    def trial(direction):
        calls.append(direction)
        node.trials.append({'direction':direction,'qualification_eligible':qualified})
        return qualified
    node.trial=trial
    if qualified:
        module.OnboardBraking.run(node)
        assert calls==list(module.DIRECTIONS)
    else:
        with pytest.raises(RuntimeError,match='sequence stopped'):
            module.OnboardBraking.run(node)
        assert calls==['forward']


@pytest.mark.parametrize('selection,nominal_only',[(['--fault','scan_disconnect'],False),(['--attended-sequence'],False),(['--attended-sequence','--nominal-only'],True),(['--faults-only'],False)])
def test_stage_020_fault_selection_is_not_skipped(monkeypatch,tmp_path,selection,nominal_only):
    import shutil
    import sys
    from pathlib import Path
    module=import_module('test-onboard-braking')
    config=tmp_path/'config'
    config.mkdir()
    for name in ('nav2_params.yaml','onboard_braking.yaml','base_speed_qualification.yaml'):
        shutil.copyfile(Path(__file__).parents[1]/'config'/name,config/name)
    monkeypatch.setattr(module,'ROOT',tmp_path)
    monkeypatch.setattr(sys,'argv',['test-onboard-braking.py','--payload-g','200','--stage','0.20',*selection])
    monkeypatch.setattr(module.NAV,'installed_stack_arguments',lambda:['remote_ip:=robot-1'])
    monkeypatch.setattr(module.subprocess,'check_output',lambda *a,**k:'123\n')
    profiles=[]
    def launch(node,output,**kwargs):
        import yaml
        profiles.append(yaml.safe_load((output/'profile.yaml').read_text()))
        assert kwargs['production'] is False
    monkeypatch.setattr(module.NAV,'main',launch)
    module.main()
    assert profiles[0]['nominal_only'] is nominal_only
    assert profiles[0]['trials_per_direction']==1
    assert profiles[0]['faults_only'] is ('--faults-only' in selection)


def test_faults_only_batches_all_axes_without_running_nominals(monkeypatch):
    from types import SimpleNamespace
    module=import_module('test-onboard-braking')
    monkeypatch.setattr(module.subprocess,'check_output',lambda *a,**k:'123\n')
    names=[]
    node=SimpleNamespace(config={'stage':'0.20','body_radius_m':.33,'faults_only':True,
        'nominal_only':False,'linear_speed_m_s':.2,'angular_speed_rad_s':.4,'depth_filter_pid':'123'},
        pose=(0.,0.,0.),ranges=[{'pose':[0.,0.,0.]}]*21,range_info=[{}],camera_poses=[{}],
        views={'front':1,'wrist':1,'astra':1},trials=[],checks={},
        wait_ready=lambda:None,wait=lambda condition,timeout:condition(),move=lambda *a,**k:None,
        pulse_start=lambda direction:direction,save=lambda:None)
    def fault(name,*args):
        names.append((node.angular_test,name))
        node.checks[name]={'passed':True}
    node.fault=fault
    module.OnboardBraking.run(node)
    assert names==[(angular,name) for angular in (False,True) for name in
        ('scan_disconnect','compute_command_loss','depth_disconnect','telemetry_loss')]
    assert len(node.checks)==8


def test_return_only_stops_after_the_fixed_center_recovery():
    from types import SimpleNamespace
    module=import_module('test-onboard-braking')
    calls=[]
    node=SimpleNamespace(config={'stage':'0.30','return_only':True,'body_radius_m':.33,
        'test_center':(0.,0.,0.)},pose=(0.,0.,0.),ranges=[{'pose':[-.25,0.,0.]}]*21,
        range_info=[{}],camera_poses=[{}],views={'front':1,'wrist':1,'astra':1},
        wait_ready=lambda:None,wait=lambda condition,timeout:condition(),
        move=lambda target,**limits:calls.append(target),save=lambda:calls.append('saved'))
    module.OnboardBraking.run(node)
    assert calls==[(0.,0.,0.),'saved']


def test_fault_boundary_uses_the_same_pose_frame_as_the_center(monkeypatch):
    import time
    module=import_module('test-onboard-braking')
    monkeypatch.setattr(module.NAV.Test,'tick',lambda *a,**k:None)
    node=module.FAULT.FaultTest.__new__(module.FAULT.FaultTest)
    node.deadline,node.center=time.monotonic()+10,(0.,0.,0.)
    node.maximum_center_radius_m,node.pose=.4,(-.414,0.,0.)
    with pytest.raises(RuntimeError,match='early center'):
        module.FAULT.FaultTest.tick(node)
    module.FAULT.FaultTest.tick(node,pose_source=lambda:(-.25,0.,0.))
    with pytest.raises(RuntimeError,match='early center'):
        module.FAULT.FaultTest.tick(node,pose_source=lambda:(-.41,0.,0.))


def test_scan_geometry_recovers_motion_and_rejects_a_single_wall():
    module=import_module('test-onboard-braking')
    fit=module.register_scan
    x=np.linspace(-1.,1.,180)
    reference=np.concatenate([np.column_stack((x,np.ones_like(x)*2)),
                              np.column_stack((np.ones_like(x)*-2,x)),
                              np.column_stack((np.ones_like(x)*2,x))])
    angle=.05
    c,s=np.cos(angle),np.sin(angle)
    points=(reference-[.03,-.015])@np.array([[c,-s],[s,c]])
    points+=np.random.default_rng(7).normal(0,.001,points.shape)
    pose,covariance,sensitivity=fit(reference,points,[0.,0.,0.],.33)
    assert pose==pytest.approx([.03,-.015,.05],abs=.001)
    assert 0<max(covariance)<1e-6 and sensitivity<.002
    with pytest.raises(ValueError,match='constrain'):
        fit(reference[:180],reference[:180],[0.,0.,0.],.33)


def test_independent_stop_bounds_hidden_motion_and_rejects_bad_windows():
    module=import_module('test-onboard-braking')
    measure=module.stopping_measurement
    config={'body_radius_m':.33,'maximum_frame_gap_s':.5,'point_speed_bound_m_s':.05,
            'scale_reserve_fraction':.15,'measurement_uncertainty_m':.015,
            'maximum_stopping_distance_m':.05,'maximum_stop_time_s':1.5}
    samples=[{'time':i*.2,'stamp':i*.2,'pose':[0. if i==0 else .01,0.,0.],
              'covariance':[1e-6,1e-6,1e-6]} for i in range(10)]
    result=measure(samples,.1,config,.001)
    assert result['within_budget']
    assert result['unobserved_excursion_bound_m']==pytest.approx(.005)
    assert result['conservative_swept_distance_m']==pytest.approx(.015)
    delayed=[{**s,'capture_time':s['time'],'time':s['time']+.3} for s in samples]
    assert measure(delayed,.1,config,.001)['within_budget']
    assert not measure([{**s,'covariance':[.001,.001,.001]} for s in samples],.1,config,.001)['within_budget']
    assert not measure([{**s,'pose':[s['pose'][0]*5,0.,0.]} for s in samples],.1,config,.001)['within_budget']
    for invalid in (samples[:5],samples[1:],samples[:5]+[{**samples[5],'stamp':.1}]+samples[6:]):
        with pytest.raises(ValueError):
            measure(invalid,.1,config,.001)


def test_terminal_capture_fit_uses_the_final_capture_interval():
    module=import_module('test-onboard-braking')
    samples=[{'time':i*.05,'pts_ns':int(i*50_000_000),'pose':[.02*i*.05,0.,.1*i*.05]} for i in range(25)]
    assert module.terminal_observed_speed(samples,False)==pytest.approx(.02)
    assert module.terminal_observed_speed(samples,True)==pytest.approx(.1)
    with pytest.raises(ValueError):
        module.terminal_observed_speed(samples[:3],False)
    with pytest.raises(ValueError,match='incomplete'):
        module.terminal_observed_speed(samples[:5],False)


def test_terminal_capture_fit_excludes_acceleration_with_five_hz_captures():
    module=import_module('test-onboard-braking')
    samples=[{'pts_ns':i*200_000_000,'pose':[.2*max(0.,i*.2-.3),0.,0.]}
             for i in range(8)]
    assert module.terminal_observed_speed(samples,False)==pytest.approx(.2)


def test_stopping_clearance_uses_farthest_point_excursion_without_summing_jitter():
    excursion=import_module('test-onboard-braking').maximum_swept_excursion
    assert excursion([[0,0,0],*[p for _ in range(100) for p in [[.001,0,0],[0,0,0]]]],.53)==.001
    assert excursion([[0,0,0],[.01,0,.2],[0,0,0]],.53)==pytest.approx(.01+1.06*np.sin(.1))
