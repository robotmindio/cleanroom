import runpy
from pathlib import Path

import pytest
import numpy as np


def test_scan_geometry_recovers_motion_and_rejects_a_single_wall():
    module=runpy.run_path(str(Path(__file__).parents[1]/'scripts/test-onboard-braking.py'))
    fit=module['register_scan']
    x=np.linspace(-1.,1.,180)
    reference=np.concatenate([np.column_stack((x,np.ones_like(x)*2)),
                              np.column_stack((np.ones_like(x)*-2,x)),
                              np.column_stack((np.ones_like(x)*2,x))])
    angle=.05;c,s=np.cos(angle),np.sin(angle)
    points=(reference-[.03,-.015])@np.array([[c,-s],[s,c]])
    points+=np.random.default_rng(7).normal(0,.001,points.shape)
    pose,covariance,sensitivity=fit(reference,points,[0.,0.,0.],.33)
    assert pose==pytest.approx([.03,-.015,.05],abs=.001)
    assert 0<max(covariance)<1e-6 and sensitivity<.002
    with pytest.raises(ValueError,match='constrain'):
        fit(reference[:180],reference[:180],[0.,0.,0.],.33)


def test_independent_stop_bounds_hidden_motion_and_rejects_bad_windows():
    module=runpy.run_path(str(Path(__file__).parents[1]/'scripts/test-onboard-braking.py'))
    measure=module['stopping_measurement']
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
        with pytest.raises(ValueError):measure(invalid,.1,config,.001)


def test_terminal_capture_fit_and_dual_envelope():
    module=runpy.run_path(str(Path(__file__).parents[1]/'scripts/test-braking.py'))
    samples=[{'time':i*.05,'pts_ns':int(i*50_000_000),
              'pose':[.02*i*.05,0.,0.],'marker_pose':[.03*i*.05,0.,0.]} for i in range(25)]
    assert module['terminal_observed_speed'](samples,False)==pytest.approx(.02)
    assert module['terminal_observed_speed'](samples,False,'marker_pose')==pytest.approx(.03)
    result=module['dual_stop_measurement'](samples,.53,.01)
    assert result['conservative_swept_distance_m']==pytest.approx(.036)
    assert result['conservative_swept_distance_m']>=result['floor_swept_distance_m']
    with pytest.raises(ValueError):module['terminal_observed_speed'](samples[:3],False)
    with pytest.raises(ValueError):module['dual_stop_measurement']([{**s,'marker_pose':None} for s in samples],.53,.01)
