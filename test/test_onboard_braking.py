from importlib import import_module

import pytest
import numpy as np


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
    for key in ('linear_speed_m_s','angular_speed_rad_s','payload_kg',
                'measurement_uncertainty_m','maximum_stopping_distance_m'):
        with pytest.raises(ValueError,match='profile differs'):
            resume(previous,{**config,key:config[key]*2})


def test_selected_stop_does_not_retry_or_start_a_return_movement():
    from types import SimpleNamespace
    run=import_module('test-onboard-braking').OnboardBraking.run
    calls=[]
    node=SimpleNamespace(config={'direction':'forward','body_radius_m':.33,'test_center':(0.,0.,0.)},
        pose=(0.,0.,0.),ranges=[{'pose':[0.,0.,0.]}]*21,range_info=[{}],
        camera_poses=[{}],views={'front':1,'wrist':1,'astra':1},
        wait_ready=lambda:None,wait=lambda condition,timeout:condition())
    def move(target,**limits):
        assert target==(0.,0.,0.)
        assert limits=={'linear_limit':.02,'angular_limit':.06}
        calls.append('center')
        node.ranges.append({'pose':[-.06,0.,0.]})
    def trial(direction):
        assert node.origin_range==[-.06,0.,0.]
        calls.append(direction)
        return True
    node.move,node.trial=move,trial
    run(node)
    assert calls==['center','forward']
    node.trial=lambda direction:calls.append(direction) or False
    with pytest.raises(RuntimeError,match='unqualified'):
        run(node)
    assert calls==['center','forward','center','forward']


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


def test_stopping_clearance_uses_farthest_point_excursion_without_summing_jitter():
    excursion=import_module('test-onboard-braking').maximum_swept_excursion
    assert excursion([[0,0,0],*[p for _ in range(100) for p in [[.001,0,0],[0,0,0]]]],.53)==.001
    assert excursion([[0,0,0],[.01,0,.2],[0,0,0]],.53)==pytest.approx(.01+1.06*np.sin(.1))
