#!/usr/bin/env python3
"""Finite loaded stopping/fault tests; observe with robot cameras and raw LiDAR.

Source setup.bash. Requires the authorized attended 30 cm area and travel_stow.
Does not restart motor services, change torque or grant acceptance automatically.
"""
import argparse
import json
import math
from pathlib import Path
import runpy
import signal
import shlex
import subprocess
import time

import numpy as np
from scipy.spatial import cKDTree
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import OdomInfo
from sensor_msgs.msg import JointState, LaserScan, PointCloud2
from lekiwi_rmf.scan_self_filter import blank_body_sectors, parse_sectors

ROOT = Path(__file__).resolve().parents[1]
FAULT = runpy.run_path(str(ROOT/'scripts/test-physical-acceptance.py'))
NAV = FAULT['navigation']
BRAKE = runpy.run_path(str(ROOT/'scripts/test-braking.py'))
DIRECTIONS = BRAKE['DIRECTIONS']


def register_scan(reference, points, guess, radius):
    """Point-to-line SE(2) fit, with geometry covariance and sector jackknife.

    RTAB-Map's ICP covariance is a scaled correspondence residual, rather than
    an observability-aware pose covariance. Calculate the latter from the actual
    scan geometry and check sensitivity to removing whole angular sectors.
    """
    tree = cKDTree(reference)
    _, neighbors = tree.query(reference, k=7)
    patches = reference[neighbors]
    centered = patches-patches.mean(axis=1,keepdims=True)
    eigenvalues, eigenvectors = np.linalg.eigh(np.einsum('nki,nkj->nij',centered,centered))
    normals = eigenvectors[:,:,0]
    planar = eigenvalues[:,0] < .12*eigenvalues[:,1]
    pose = np.array(guess,dtype=float)
    for _ in range(20):
        c,s = math.cos(pose[2]),math.sin(pose[2])
        rotated = points@np.array([[c,s],[-s,c]])
        transformed = rotated+pose[:2]
        distances, indices = tree.query(transformed)
        valid = (distances<.06)&planar[indices]
        if valid.sum()<80:
            raise ValueError('insufficient independent wall correspondences')
        normal = normals[indices[valid]]
        residual = np.sum(normal*(transformed[valid]-reference[indices[valid]]),axis=1)
        jacobian = np.column_stack((normal,np.sum(normal*rotated[valid][:,::-1]*[-1,1],axis=1)))
        weights = np.minimum(1.,.015/np.maximum(abs(residual),1e-9))
        weighted = jacobian*np.sqrt(weights[:,None])
        scaled = weighted.copy();scaled[:,2]/=radius
        if np.linalg.cond(scaled)>30:
            raise ValueError('independent scan geometry cannot constrain all three axes')
        correction = np.linalg.lstsq(weighted,-residual*np.sqrt(weights),rcond=None)[0]
        pose += correction
        # Submillimeter convergence fits the separate 1 mm numerical reserve;
        # noisy nearest-neighbor assignments do not converge at micron scale.
        if np.linalg.norm(correction[:2])+radius*abs(correction[2])<.0005:break
    else:
        raise ValueError('independent scan alignment did not converge')
    variance = float(np.sum(weights*residual**2)/(len(residual)-3))
    covariance = variance*np.linalg.inv(weighted.T@weighted)
    sectors = (np.arctan2(reference[indices[valid],1],reference[indices[valid],0])%(2*math.pi)//(math.pi/4)).astype(int)
    sensitivity = 0.
    for sector in np.unique(sectors):
        keep = sectors!=sector
        if keep.sum()<80 or np.linalg.cond(scaled[keep])>30:
            raise ValueError('independent scan loses observability after removing a sector')
        delta = np.linalg.lstsq(weighted[keep],-residual[keep]*np.sqrt(weights[keep]),rcond=None)[0]
        sensitivity = max(sensitivity,float(np.linalg.norm(delta[:2])+radius*abs(delta[2])))
    # A block jackknife retains correlated returns; the covariance handles the
    # remaining correspondence noise. Scale/dimensional reserves are separate.
    return pose.tolist(),[float(np.linalg.eigvalsh(covariance[:2,:2])[-1])]*2+[float(covariance[2,2])],sensitivity


def stopping_measurement(samples, cut, config, stationary_jitter):
    if len(samples)<8 or samples[0].get('capture_time',samples[0]['time'])>cut:
        raise ValueError('independent stop window needs a preceding sample and eight captures')
    gaps = np.diff([s['stamp'] for s in samples])
    if not np.all(gaps>0) or gaps.max()>config['maximum_frame_gap_s']:
        raise ValueError('independent capture timestamps are discontinuous')
    radius = config['body_radius_m']
    poses = [s['pose'] for s in samples]
    sweep = BRAKE['maximum_swept_excursion'](poses,radius)
    # A hidden excursion between bounded-speed endpoints needs travel out and
    # back. Lipschitz continuity bounds it by speed * capture gap / 2.
    blind = config['point_speed_bound_m_s']*float(gaps.max())/2
    sigma = max(3*(math.sqrt(max(s['covariance'][0],s['covariance'][1]))+
                   radius*math.sqrt(s['covariance'][2])) for s in samples)
    error = stationary_jitter+sigma+max(s.get('sector_sensitivity_m',0.) for s in samples)+config['scale_reserve_fraction']*sweep+.001
    terminal = np.median(np.array(poses[-8:]),axis=0)
    filtered = [np.median(np.array(poses[max(0,i-2):i+1]),axis=0) for i in range(len(poses))]
    outside = [i for i,p in enumerate(filtered) if math.dist(p[:2],terminal[:2])>.005
               or abs(NAV['angle'](p[2]-terminal[2]))>.015]
    stable = samples[min(max(outside)+1 if outside else 1,len(samples)-1)]
    latency = max(0.,stable['time']-cut)+float(gaps.max())
    distance = sweep+blind
    return {'observed_swept_distance_m':sweep,'unobserved_excursion_bound_m':blind,
            'conservative_swept_distance_m':distance,'uncertainty_upper_m':error,
            'maximum_capture_gap_s':float(gaps.max()),'stop_time_receive_upper_s':latency,
            'first_sample_time':samples[0]['time'],'last_sample_time':samples[-1]['time'],
            'fault_cut_time':cut,'samples':len(samples),'baseline_time_basis':'sensor_capture_stamp',
            'within_budget':distance+config['measurement_uncertainty_m']<=config['maximum_stopping_distance_m']
              and latency<=config['maximum_stop_time_s'] and error<=config['measurement_uncertainty_m']}


class OnboardBraking(FAULT['FaultTest']):
    def __init__(self, output, config):
        super().__init__(output=output)
        self.config = config
        self.deadline = time.monotonic()+config['maximum_runtime_s']
        self.ranges,self.range_info,self.camera_poses,self.sources = [],[],[],{}
        self.source_stamps={}
        self.native_poses,self.scans,self.reference_points = [],[],None
        self.reference_scans=[]
        mask=yaml.safe_load((ROOT/'config/lidar_self_mask.yaml').read_text())['scan_self_filter']['ros__parameters']
        self.sectors=parse_sectors(*(mask[k] for k in ('body_start_deg','body_end_deg','body_max_range_m')))
        self.create_subscription(LaserScan,'/pi/lidar/scan',self.direct_scan,NAV['qos_profile_sensor_data'])
        self.origin_range = None
        self.stationary_jitter = 0.
        self.trials = []
        self.create_subscription(Odometry,'/verification/loaded_range_odometry',self.range_pose,NAV['qos_profile_sensor_data'])
        self.create_subscription(OdomInfo,'/verification/loaded_range_info',
            lambda m:self.range_info.append({'time':time.monotonic(),'lost':m.lost,'ratio':m.icp_inliers_ratio}),NAV['qos_profile_sensor_data'])
        self.create_subscription(Odometry,'/verification/loaded_camera_odometry',
            lambda m:self.camera_poses.append(self.pose_row(m)),NAV['qos_profile_sensor_data'])
        for topic,kind in [('/scan',LaserScan),('/camera/depth/points',PointCloud2),('/joint_states',JointState)]:
            self.create_subscription(kind,topic,lambda m,t=topic:self.source_sample(t,m),NAV['qos_profile_sensor_data'])
        self.observers=[]
        for binary,name,topic,odom,info,parameters in [
            ('icp_odometry','loaded_braking_range','/pi/lidar/scan','loaded_range_odometry','loaded_range_info',
             ['-p','Reg/Force3DoF:="true"','-p','Icp/PointToPlane:="true"','-p','Icp/RangeMin:="0.35"','-p','Icp/RangeMax:="5.0"']),
            ('rgbd_odometry','loaded_braking_camera','/slam/astra/rgbd_image','loaded_camera_odometry','loaded_camera_info',
             ['-p','subscribe_rgbd:=true'])]:
            log=(output/(name+'.log')).open('w')
            args=[f'/opt/ros/jazzy/lib/rtabmap_odom/{binary}','--ros-args','-r',f'__node:={name}',
                  '-r',f'{"scan" if binary=="icp_odometry" else "rgbd_image"}:={topic}',
                  '-r',f'odom:=/verification/{odom}','-r',f'odom_info:=/verification/{info}',
                  '-p','frame_id:=base_footprint','-p','publish_tf:=false','-p','qos:=1',
                  '-p','Odom/ResetCountdown:="0"',*parameters]
            self.observers.append((subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT),log))

    def pose_row(self,message):
        p=message.pose.pose
        stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        return {'time':time.monotonic(),'stamp':stamp,
                'age':self.get_clock().now().nanoseconds/1e9-stamp,
                'pose':[p.position.x,p.position.y,NAV['yaw'](p.orientation)],
                'covariance':[message.pose.covariance[i] for i in (0,7,35)]}

    def range_pose(self,message):
        row=self.pose_row(message)
        if not all(math.isfinite(x) for x in [*row['pose'],*row['covariance'],row['age']]) or min(row['covariance'])<0:
            raise RuntimeError('invalid independent raw-LiDAR pose')
        self.native_poses.append(row)

    def direct_scan(self,message):
        scan=blank_body_sectors(message,self.sectors)
        ranges=np.array(scan.ranges);angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
        valid=np.isfinite(ranges)&(ranges>.2)&(ranges<5.)
        points=np.column_stack((ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid])))
        if len(points)<100:raise RuntimeError('too few independent raw-LiDAR points')
        if not self.buffer.can_transform('base_footprint',scan.header.frame_id,NAV['Time']()):return
        transform=self.buffer.lookup_transform('base_footprint',scan.header.frame_id,NAV['Time']()).transform
        heading=NAV['yaw'](transform.rotation);c,s=math.cos(heading),math.sin(heading)
        points=points@np.array([[c,s],[-s,c]])+[transform.translation.x,transform.translation.y]
        stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        self.scans.append({'stamp':stamp,'time':time.monotonic(),'points':points.tolist()})
        if self.reference_points is None:
            self.reference_scans.append(points)
            if len(self.reference_scans)<10:return
            # A stationary median reference prevents noise in one initial scan
            # from becoming a persistent wall/heading error in every later fit.
            cloud=np.concatenate(self.reference_scans)
            bins=(np.degrees(np.arctan2(cloud[:,1],cloud[:,0]))%360).astype(int)
            self.reference_points=np.array([np.median(cloud[bins==key],axis=0) for key in np.unique(bins) if (bins==key).sum()>=5])
        pose,covariance,sensitivity=register_scan(self.reference_points,points,self.ranges[-1]['pose'] if self.ranges else [0.,0.,0.],self.config['body_radius_m'])
        received=time.monotonic();age=self.get_clock().now().nanoseconds/1e9-stamp
        self.ranges.append({'time':received,'stamp':stamp,'age':age,'capture_time':received-age,
            'pose':pose,'covariance':covariance,'sector_sensitivity_m':sensitivity})

    def source_sample(self,topic,message):
        stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        self.sources[topic]=time.monotonic()-(self.get_clock().now().nanoseconds/1e9-stamp)
        self.source_stamps[topic]=stamp

    def tick(self,twist=None,check=True):
        super().tick(twist,check)
        if check and self.active:
            def fresh():
                return self.ranges and time.monotonic()-self.ranges[-1]['time']+self.ranges[-1]['age']<=.5
            if not fresh():
                if self.phase:
                    self.command.publish(Twist())
                    raise RuntimeError('independent raw-LiDAR tracking failed during the fault')
                self.pause_until(fresh,'fresh independent raw-LiDAR capture')
            if self.origin_range and math.dist(self.ranges[-1]['pose'][:2],self.origin_range[:2])>=.16:
                self.command.publish(Twist())
                raise RuntimeError('independent early 16 cm boundary reached')

    def observe_stop(self,cut,end=None,cut_stamp=None):
        rows=self.ranges if end is None else self.ranges[:end]
        before=[i for i,r in enumerate(rows) if (r['stamp']<=cut_stamp if cut_stamp is not None else r['capture_time']<=cut)]
        if not before:
            raise RuntimeError('no independent pose precedes the stop')
        selected=rows[before[-1]:]
        # An exact matching scan/depth capture stamp shares the device clock;
        # avoid a microsecond conversion difference selecting an older scan.
        if cut_stamp is not None:selected=[{**r,'capture_time':cut+(r['stamp']-cut_stamp)} for r in selected]
        return stopping_measurement(selected,cut,self.config,self.stationary_jitter)

    def save(self):
        (self.output/'measurements.json').write_text(json.dumps({
            'trials':self.trials,'faults':self.checks,'profile':self.config,
            'stationary_jitter_m':self.stationary_jitter,'flags':self.flags,'health':self.health,
            'physical_acceptance_granted':False},indent=2)+'\n')

    def trial(self,direction):
        self.move(self.center,linear_limit=.02,angular_limit=.06)
        self.wait(lambda:self.flags.get('base_motion_permitted'),15)
        linear,angular=NAV['load_base_speed_limits'](ROOT/'config/nav2_params.yaml')
        speed=angular if direction.startswith('rotation') else linear
        self.angular_test=direction.startswith('rotation')
        command=Twist();command.linear.x,command.linear.y,command.angular.z=[float(speed*v) for v in DIRECTIONS[direction]]
        start=time.monotonic();begin=len(self.ranges);faults=len(self.health_faults);pauses=self.motion_pauses
        while time.monotonic()-start<2.5:
            if not self.flags.get('base_motion_permitted'):
                self.command.publish(Twist());raise RuntimeError('nominal permission withdrawn')
            self.tick(command)
        cut=time.monotonic();self.command.publish(Twist())
        # Acceptance covers the configured production maximum. Servo speed
        # quantization and ground slip can make physical motion slower; retain
        # that measurement without demanding a command above the production cap.
        covered=self.safe_speed is not None and self.safe_speed>=speed*.9 and self.measured_speed>=(.015 if self.angular_test else .005) and time.monotonic()-self.odom_at<.3
        wheel_speed,guarded_speed=self.measured_speed,self.safe_speed
        terminal=[{**r,'pts_ns':int(r['stamp']*1e9)} for r in self.ranges[begin:]]
        observed=BRAKE['terminal_observed_speed'](terminal,direction.startswith('rotation'),window_s=1.2)
        until=time.monotonic()+2.5
        while time.monotonic()<until:self.tick(Twist())
        try:
            result=self.observe_stop(cut)
        except ValueError as error:
            # Keep completed measurements when a later observation window has
            # a gap. This attempt is recorded and excluded, never approved.
            self.trials.append({'direction':direction,'requested_speed':speed,
                'fault_cut_time':cut,'qualification_eligible':False,'measurement_error':str(error)})
            self.save();print('UNQUALIFIED STOP',direction,str(error),flush=True)
            return False
        result.update(direction=direction,requested_speed=speed,terminal_observed_speed=observed,
                      requested_speed_covered=covered and observed>=(.015 if self.angular_test else .005),
                      speed_coverage_basis='maximum production guarded command, with fresh wheel feedback and independent ground motion',
                      terminal_wheel_speed=wheel_speed,terminal_guarded_speed=guarded_speed,
                      feedback_interrupted=len(self.health_faults)>faults or self.motion_pauses>pauses)
        result['qualification_eligible']=result['within_budget'] and result['requested_speed_covered'] and not result['feedback_interrupted']
        self.trials.append(result);self.save();print('STOP',json.dumps(result),flush=True)
        if not result['within_budget']:
            if result['conservative_swept_distance_m']+max(result['uncertainty_upper_m'],self.config['measurement_uncertainty_m'])>self.config['maximum_stopping_distance_m'] or result['stop_time_receive_upper_s']>self.config['maximum_stop_time_s']:
                raise RuntimeError('loaded physical stopping bound exceeded')
            print('UNQUALIFIED STOP',direction,'measurement uncertainty exceeds the declared limit',flush=True)
        return result['qualification_eligible']

    def fault(self,name,begin,restore,expected):
        captured={}
        def inject(command):
            captured['cut']=time.monotonic();begin(command)
        def recover():
            try:
                captured['end']=len(self.ranges)
                topic={'scan_disconnect':'/scan','depth_disconnect':'/camera/depth/points','telemetry_loss':'/joint_states'}.get(name)
                if topic:captured['cut']=self.sources[topic]
                captured['measurement']=self.observe_stop(captured['cut'],captured['end'],self.source_stamps[topic] if topic else None)
            finally:
                restore()
        super().fault(name,inject,recover,expected)
        self.checks[name]['independent']=captured['measurement'];self.save()
        print('FAULT STOP',name,json.dumps(captured['measurement']),flush=True)
        if not captured['measurement']['within_budget']:raise RuntimeError('loaded fault stopping/error bound exceeded')

    def run(self):
        self.wait_ready();self.wait(lambda:len(self.ranges)>20 and self.range_info and self.camera_poses and len(self.views)==3,25)
        self.center=tuple(self.config['test_center']) if self.config.get('test_center') else self.pose
        self.origin_range=self.ranges[-1]['pose'];self.active=True
        poses=[r['pose'] for r in self.ranges[-15:]]
        self.stationary_jitter=max(BRAKE['maximum_swept_excursion'](poses[i:],self.config['body_radius_m']) for i in range(len(poses)))
        for direction in DIRECTIONS:
            completed=sum(t['direction']==direction and t['qualification_eligible'] for t in self.trials);attempts=0
            while completed<self.config['trials_per_direction']:
                attempts+=1
                if attempts>10:raise RuntimeError('too many unqualified stopping attempts: '+direction)
                completed+=int(self.trial(direction))
        self.move(self.center,linear_limit=.02,angular_limit=.06)
        if self.config['nominal_only']:return
        driver=subprocess.check_output(['pgrep','-f','/lib/lekiwi_rmf/lekiwi_driver '],text=True).split()
        scan=subprocess.check_output(['pgrep','-f','/lib/lekiwi_rmf/scan_self_filter '],text=True).split()
        if len(driver)!=1 or len(scan)!=1:raise RuntimeError('expected exactly one managed driver and scan filter')
        for angular in (False,True):
            self.angular_test=angular;self.test_speed=self.config['angular_speed_rad_s' if angular else 'linear_speed_m_s']
            axis='angular' if angular else 'linear'
            for label,pid,reason in [('scan_disconnect',scan[0],'scan:'),('compute_command_loss',driver[0],'driver:')]:
                timer=f'lekiwi-loaded-{label}-{axis}'
                def stop(command,pid=pid,timer=timer):
                    self.action(['systemd-run','--user','--quiet','--collect',f'--unit={timer}','--on-active=8s','/usr/bin/kill','-CONT',pid],command)
                    self.action(['kill','-STOP',pid],command)
                def resume(pid=pid,timer=timer):
                    self.action(['kill','-CONT',pid],Twist())
                    self.action(['systemctl','--user','stop',timer+'.timer'],Twist())
                self.fault(label,stop,resume,reason)
                self.checks[axis+'/'+label]=self.checks.pop(label);self.save()
            def pause_depth(command):
                self.remote(['sudo','-n','systemd-run','--quiet','--collect','--unit=lekiwi-loaded-depth-restore',
                    '--on-active=8s','/usr/bin/kill','-CONT',self.config['depth_filter_pid']],command)
                self.remote(['kill','-STOP',self.config['depth_filter_pid']],command)
            def resume_depth():
                self.remote(['kill','-CONT',self.config['depth_filter_pid']],Twist())
                self.remote(['sudo','-n','systemctl','stop','lekiwi-loaded-depth-restore.timer'],Twist())
            self.fault('depth_disconnect',pause_depth,resume_depth,'depth:')
            self.checks[axis+'/depth_disconnect']=self.checks.pop('depth_disconnect');self.save()
            table='lekiwi_loaded_acceptance'
            def block(command):
                self.remote(['sudo','-n','systemd-run','--quiet','--collect','--unit=lekiwi-loaded-telemetry-restore','--on-active=12s','/usr/sbin/nft','delete','table','inet',table],command)
                rules=f'add table inet {table}\nadd chain inet {table} output {{ type filter hook output priority 0; policy accept; }}\nadd rule inet {table} output tcp sport 5556 drop\n'
                self.remote(['sudo','-n','/usr/sbin/nft','-f','-'],command,rules)
            def unblock():
                self.remote(['sudo','-n','/usr/sbin/nft','delete','table','inet',table],Twist())
                self.remote(['sudo','-n','systemctl','stop','lekiwi-loaded-telemetry-restore.timer'],Twist())
            self.fault('telemetry_loss',block,unblock,'driver:')
            self.checks[axis+'/telemetry_loss']=self.checks.pop('telemetry_loss');self.save()

    def destroy_node(self):
        try:
            self.save()
            (self.output/'independent-poses.json').write_text(json.dumps({'ranges':self.ranges,'native_poses':self.native_poses,'scans':self.scans,'range_info':self.range_info,'camera_poses':self.camera_poses},indent=2)+'\n')
        finally:
            for process,log in self.observers:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:process.wait(timeout=8)
                    except subprocess.TimeoutExpired:process.kill();process.wait()
                log.close()
            super().destroy_node()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--payload-g',type=float,required=True)
    parser.add_argument('--nominal-only',action='store_true')
    parser.add_argument('--resume',type=Path,help='retain qualified trials and the fixed center from this earlier loaded run')
    parser.add_argument('--center',type=float,nargs=3,metavar=('X','Y','YAW'),help='clear test center in wheel coordinates; the existing 12 cm relocation guard still applies')
    args=parser.parse_args()
    if not math.isfinite(args.payload_g) or args.payload_g<0:parser.error('payload must be finite and nonnegative')
    config=yaml.safe_load((ROOT/'config/onboard_braking.yaml').read_text())
    linear,angular=NAV['load_base_speed_limits'](ROOT/'config/nav2_params.yaml')
    config.update(payload_kg=args.payload_g/1000,linear_speed_m_s=linear,angular_speed_rad_s=angular,
                  nominal_only=args.nominal_only,point_speed_bound_m_s=1.15*(linear+config['body_radius_m']*angular))
    device=next(a.partition(':=')[2] for a in NAV['installed_stack_arguments']() if a.startswith('remote_ip:='))
    # The stream filter publishes required safety depth. Its raw camera remains
    # powered; pausing it exercises missing input without resetting the USB hub.
    remote='pgrep -f '+shlex.quote('^python3 .*/lib/lekiwi_rmf/astra_cloud_filter ')
    candidates=subprocess.check_output(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5',device,remote],text=True).split()
    if len(candidates)!=1 or not candidates[0].isdigit():raise RuntimeError('expected exactly one device depth filter')
    config['depth_filter_pid']=candidates[0]
    resumed=[]
    if args.resume:
        previous=json.loads((args.resume/'measurements.json').read_text())
        if previous['profile']['payload_kg']!=config['payload_kg'] or previous['profile']['measurement_uncertainty_m']!=config['measurement_uncertainty_m']:
            raise ValueError('resumed load/measurement profile differs')
        config['test_center']=json.loads((args.resume/'result.json').read_text())['origin']
        if not config['test_center']:raise ValueError('resumed test center is missing')
        resumed=[{**t,'source_run':t.get('source_run',str(args.resume))} for t in previous['trials'] if t['qualification_eligible']]
    if args.center:
        if not all(math.isfinite(v) for v in args.center):parser.error('test center must be finite')
        config['test_center']=args.center
    output=ROOT/'.benchmarks/onboard-braking'/time.strftime('%Y%m%d-%H%M%S');output.mkdir(parents=True)
    (output/'profile.yaml').write_text(yaml.safe_dump(config))
    def node():
        instance=OnboardBraking(output,config);instance.trials=resumed;return instance
    NAV['main'](node,output,production=True,payload_kg=config['payload_kg'])


if __name__=='__main__':main()
