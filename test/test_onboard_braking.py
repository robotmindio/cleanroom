from importlib import import_module

import pytest
import numpy as np


def test_nominal_permission_loss_stops_and_reports_the_gate():
    from types import SimpleNamespace
    module=import_module('test-onboard-braking')
    commands=[]
    node=SimpleNamespace(move=lambda target:None,pulse_start=lambda direction:(0.,0.,0.),
        wait=lambda condition,timeout:None,config={'linear_speed_m_s':.25,'angular_speed_rad_s':.5},
        ranges=[],health_faults=[],joint_samples=[],motion_pauses=0,flags={'base_motion_permitted':False,'arm_stowed':False},
        health={'arm_stowed':'false','faults':''},command=SimpleNamespace(publish=commands.append))
    with pytest.raises(RuntimeError,match='nominal permission withdrawn') as error:
        module.OnboardBraking.trial(node,'forward')
    assert '"arm_stowed": false' in str(error.value)
    assert '"faults": ""' in str(error.value)
    assert len(commands)==1
    assert commands[0].linear.x==commands[0].linear.y==commands[0].angular.z==0.


def test_joint_observation_preserves_the_capture_stamp_and_positions(monkeypatch):
    from types import SimpleNamespace
    from sensor_msgs.msg import JointState
    module=import_module('test-onboard-braking')
    monkeypatch.setattr(module.time,'monotonic',lambda:20.)
    message=JointState()
    message.header.stamp.sec=10
    message.name=['arm_shoulder_lift']
    message.position=[-1.7967]
    node=SimpleNamespace(sources={},source_stamps={},joint_samples=[],
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10100000000)))
    module.OnboardBraking.source_sample(node,'/joint_states',message)
    assert node.joint_samples==[{'stamp':10.,'capture_time':19.9,
        'positions':{'arm_shoulder_lift':-1.7967}}]


def test_rejected_independent_scan_records_the_error_without_creating_a_pose(monkeypatch):
    import time
    from types import SimpleNamespace
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import LaserScan
    module=import_module('test-onboard-braking')
    scan=LaserScan()
    scan.header.stamp.sec=10
    scan.angle_min=-1.5
    scan.angle_increment=.02
    scan.range_min=.1
    scan.range_max=12.
    scan.ranges=[2.]*160
    previous={'pose':[0.,0.,0.],'stamp':9.8}
    warnings=[]
    node=SimpleNamespace(sectors=[],scans=[],rejected_scans=[],ranges=[previous],
        reference_points=np.zeros((100,2)),config={'body_radius_m':.33},odom_at=time.monotonic()-1,
        buffer=SimpleNamespace(can_transform=lambda *a:True,
            lookup_transform=lambda *a:TransformStamped()),
        get_logger=lambda:SimpleNamespace(warning=warnings.append))
    def reject(*args):
        raise ValueError('independent scan loses observability after removing a sector')
    monkeypatch.setattr(module,'register_scan',reject)
    module.OnboardBraking.direct_scan(node,scan)
    assert node.ranges==[previous]
    assert node.rejected_scans==[{'stamp':10.,'error':'independent scan loses observability after removing a sector'}]
    assert len(node.scans)==len(warnings)==1


def test_a_fault_stops_when_rejected_scans_leave_the_last_pose_stale(monkeypatch):
    import time
    from types import SimpleNamespace
    module=import_module('test-onboard-braking')
    monkeypatch.setattr(module.FAULT.FaultTest,'tick',lambda *a,**k:None)
    commands=[]
    node=object.__new__(module.OnboardBraking)
    node.config={'stage':'0.25'}
    node.active=True
    node.phase='scan_disconnect'
    node.ranges=[{'pose':[0.,0.,0.],'time':time.monotonic()-.51,'age':0.}]
    node.command=SimpleNamespace(publish=commands.append)
    node.origin_range=None
    with pytest.raises(RuntimeError,match='tracking failed during the fault'):
        module.OnboardBraking.tick(node)
    assert len(commands)==1
    assert commands[0].linear.x==commands[0].linear.y==commands[0].angular.z==0.


@pytest.mark.parametrize('center',[[0.,0.,0.],[True,0.,0.],[0.,float('nan'),0.],[0.,0.]])
def test_reference_recovery_uses_the_saved_center_when_the_live_report_is_missing(monkeypatch,tmp_path,center):
    import json
    import shutil
    import sys
    from pathlib import Path
    import yaml
    module=import_module('test-onboard-braking')
    config=tmp_path/'config'
    config.mkdir()
    for name in ('nav2_params.yaml','onboard_braking.yaml','base_speed_qualification.yaml'):
        shutil.copyfile(Path(__file__).parents[1]/'config'/name,config/name)
    reference=tmp_path/'reference'
    reference.mkdir()
    (reference/'profile.yaml').write_text(yaml.safe_dump({'test_center':center}))
    (reference/'independent-poses.json').write_text(json.dumps({
        'reference_points':[[1.,0.]]*100,'ranges':[{'pose':[.1,0.,0.]}]}))
    monkeypatch.setattr(module,'ROOT',tmp_path)
    monkeypatch.setattr(sys,'argv',['test-onboard-braking.py','--payload-g','200','--stage','0.25',
        '--return-only','--reference-run',str(reference)])
    monkeypatch.setattr(module.NAV,'installed_stack_arguments',lambda:['remote_ip:=robot-1'])
    monkeypatch.setattr(module.subprocess,'check_output',lambda *a,**k:'123\n')
    profiles=[]
    monkeypatch.setattr(module.NAV,'main',lambda node,output,**kwargs:
        profiles.append(yaml.safe_load((output/'profile.yaml').read_text())))
    if center==[0.,0.,0.]:
        module.main()
        assert profiles[0]['test_center']==center
        assert profiles[0]['reference_start_pose']==[.1,0.,0.]
    else:
        with pytest.raises(ValueError,match='saved test center'):
            module.main()
        assert profiles==[]


def test_short_qualification_pulses_are_centered_in_the_fixture():
    from types import SimpleNamespace
    pulse=import_module('test-onboard-braking').OnboardBraking.pulse_start
    node=SimpleNamespace(center=(0.,0.,np.pi/2),config={'stage':'0.30',
        'nominal_command_duration_s':1.2,'linear_speed_m_s':.3,'angular_speed_rad_s':.6})
    assert pulse(node,'forward')==pytest.approx((0.,-.18,np.pi/2))
    assert pulse(node,'reverse')==pytest.approx((0.,.18,np.pi/2))
    assert pulse(node,'left')==pytest.approx((.18,0.,np.pi/2))
    assert pulse(node,'rotation_ccw')==pytest.approx((0.,0.,np.pi/2-.36))


@pytest.mark.parametrize('direction',['forward','reverse','left','right'])
def test_corridor_body_directions_all_travel_along_the_saved_forward_axis(direction):
    import math
    from types import SimpleNamespace
    module=import_module('test-onboard-braking')
    node=SimpleNamespace(center=(.2,0.,.3),config={'stage':'0.25','corridor':True,
        'nominal_command_duration_s':1.6,'linear_speed_m_s':.25,'angular_speed_rad_s':.5})
    x,y,heading=module.OnboardBraking.pulse_start(node,direction)
    assert (x,y)==pytest.approx((.2-.2*math.cos(.3),-.2*math.sin(.3)))
    ux,uy,_=module.DIRECTIONS[direction]
    assert (math.cos(heading)*ux-math.sin(heading)*uy,
            math.sin(heading)*ux+math.cos(heading)*uy)==pytest.approx((math.cos(.3),math.sin(.3)))


@pytest.mark.parametrize('permitted',[True,False])
def test_corridor_turns_in_place_at_the_stage_cap_and_stops_on_permission_loss(monkeypatch,permitted):
    import math
    module=import_module('test-onboard-braking')
    calls,commands,clock=[],[],[0.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    node=object.__new__(module.OnboardBraking)
    node.config={'corridor':True,'angular_speed_rad_s':.5,'return_linear_speed_m_s':.04,'return_angular_speed_rad_s':.06}
    node.ranges=[{'pose':[.2,0.,0.]}]
    node.flags={'base_motion_permitted':permitted}
    def translate(self,target,**kwargs):
        calls.append(target)
        assert target==(.2,0.,0.)
    monkeypatch.setattr(module.NAV.Test,'move',translate)
    def tick(command):
        assert command.linear.x==command.linear.y==0.
        assert abs(command.angular.z)<=.5
        commands.append(command)
        node.ranges[-1]['pose'][2]+=command.angular.z*.05
        clock[0]+=.05
    node.tick=tick
    node.stop=lambda:calls.append('stop')
    def wait(condition,timeout):
        assert timeout==3
        if not condition():
            raise RuntimeError('permission did not recover')
    node.wait=wait
    if permitted:
        node.move((.2,0.,math.pi))
        assert abs(module.NAV.angle(math.pi-node.ranges[-1]['pose'][2]))<.03
        assert max(abs(c.angular.z) for c in commands)==.5
    else:
        with pytest.raises(RuntimeError,match='permission did not recover'):
            node.move((.2,0.,math.pi))
        assert commands==[]
    assert calls[-1]=='stop'


@pytest.mark.parametrize('ground_speed',[.20,.24])
def test_shorter_fault_cruise_still_requires_independent_speed_coverage(monkeypatch,ground_speed):
    from types import SimpleNamespace
    from geometry_msgs.msg import Twist
    module=import_module('test-onboard-braking')
    clock, injections, zeros=[0.],[],[]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    node=object.__new__(module.OnboardBraking)
    node.config={'nominal_command_duration_s':1.6}
    node.ranges=[{'capture_time':0.}]
    node.checks={'scan_disconnect':{}}
    node.test_speed=.25
    node.angular_test=False
    node.wait=lambda condition,timeout:None
    node.command=SimpleNamespace(publish=zeros.append)
    def tick(command):
        assert command.linear.x==.25
        clock[0]+=.2
        node.ranges.append({'stamp':clock[0],'pose':[ground_speed*clock[0],0.,0.]})
    node.tick=tick
    def parent_fault(self,name,inject,*args):
        command=Twist()
        command.linear.x=.25
        inject(command)
    monkeypatch.setattr(module.FAULT.FaultTest,'fault',parent_fault)
    def begin(command):
        injections.append(clock[0])
        raise RuntimeError('mock physical injection')
    error='mock physical injection' if ground_speed>=.225 else 'did not attain independent ground speed'
    with pytest.raises(RuntimeError,match=error):
        node.fault('scan_disconnect',begin,lambda:None,'scan:')
    if ground_speed>=.225:
        assert injections==pytest.approx([1.2])
        assert node.checks['scan_disconnect']['terminal_observed_speed']==pytest.approx(ground_speed)
    else:
        assert injections==[] and len(zeros)==1
        assert zeros[0].linear.x==zeros[0].angular.z==0.


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
@pytest.mark.parametrize('stage,trials',[('0.20',1),('0.25',5),('0.30',5)])
def test_stage_fault_selection_is_not_skipped(monkeypatch,tmp_path,selection,nominal_only,stage,trials):
    import shutil
    import sys
    from pathlib import Path
    module=import_module('test-onboard-braking')
    config=tmp_path/'config'
    config.mkdir()
    for name in ('nav2_params.yaml','onboard_braking.yaml','base_speed_qualification.yaml'):
        shutil.copyfile(Path(__file__).parents[1]/'config'/name,config/name)
    monkeypatch.setattr(module,'ROOT',tmp_path)
    monkeypatch.setattr(sys,'argv',['test-onboard-braking.py','--payload-g','200','--stage',stage,*selection])
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
    assert profiles[0]['trials_per_direction']==trials
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


@pytest.mark.parametrize('selected',['depth_disconnect','telemetry_loss'])
def test_remote_fault_and_recovery_use_one_connection_each_with_safe_ordering(monkeypatch,selected):
    import shlex
    from types import SimpleNamespace
    from geometry_msgs.msg import Twist
    module=import_module('test-onboard-braking')
    monkeypatch.setattr(module.subprocess,'check_output',lambda *a,**k:'123\n')
    calls=[]
    node=SimpleNamespace(config={'stage':'0.25','body_radius_m':.33,'selected_fault':selected,
        'angular_test':False,'nominal_only':False,'linear_speed_m_s':.25,'angular_speed_rad_s':.5,'depth_filter_pid':'456'},
        pose=(0.,0.,0.),ranges=[{'pose':[0.,0.,0.]}]*21,range_info=[{}],camera_poses=[{}],
        views={'front':1,'wrist':1,'astra':1},trials=[],checks={},
        wait_ready=lambda:None,wait=lambda condition,timeout:condition(),move=lambda *a,**k:None,
        pulse_start=lambda direction:direction,save=lambda:None,
        remote=lambda argv,command,data=None:calls.append((argv,data)))
    def fault(name,begin,restore,reason):
        assert name==selected and reason==('depth:' if selected=='depth_disconnect' else 'driver:')
        begin(Twist())
        restore()
        node.checks[name]={'passed':True}
    node.fault=fault
    module.OnboardBraking.run(node)
    assert len(calls)==2 and all(argv[:2]==['bash','-c'] for argv,_ in calls)
    arm,pause=map(shlex.split,calls[0][0][2].split(' && '))
    assert arm[:3]==['sudo','-n','systemd-run']
    resume,clear=map(shlex.split,calls[1][0][2].split(' && '))
    if selected=='depth_disconnect':
        assert '--on-active=8s' in arm and arm[-3:]==['/usr/bin/kill','-CONT','456']
        assert pause==['kill','-STOP','456'] and resume==['kill','-CONT','456']
        assert clear==['sudo','-n','systemctl','stop','lekiwi-loaded-depth-restore.timer']
    else:
        assert '--on-active=12s' in arm
        assert arm[-5:]==['/usr/sbin/nft','destroy','table','inet','lekiwi_loaded_acceptance']
        assert pause==['sudo','-n','/usr/sbin/nft','-f','-']
        assert 'tcp sport 5556 drop' in calls[0][1]
        assert resume==['sudo','-n','/usr/sbin/nft','destroy','table','inet','lekiwi_loaded_acceptance']
        assert clear==['sudo','-n','systemctl','stop','lekiwi-loaded-telemetry-restore.timer']


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
