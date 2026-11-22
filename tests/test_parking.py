"""Analytic geometry, planner and interlock fixtures. No hardware or ROS publishers."""
import asyncio
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import subprocess
from unittest.mock import AsyncMock

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/rc_car'))
from rc_car.core import CommandManager, load_config

try:
    from laptop.parking.__main__ import calculate, measured, read_map, Worker
    from laptop.parking.planner import HybridAStar, Pose, Rig, World
except ModuleNotFoundError as exc:
    if exc.name not in ('scipy', 'numpy'):
        raise
    Rig = None


def config():
    cfg = load_config(ROOT / 'config/vehicle.yaml')
    cfg['geometry'].update(verified=True, wheelbase_m=.32, track_m=.22)
    cfg['servo'].update(max_angle_rad=.45, slew_rad_s=.6)
    cfg['hardware']['max_pwm'] = .2
    for m in cfg['hardware']['motors']:
        m['max_rpm'] = 60
    cfg['parking'].update(hitch_back_m=.08, drawbar_m=.42, car_front_m=.4, car_back_m=.1,
                          car_width_m=.18, trailer_front_m=.30, trailer_back_m=.12, trailer_width_m=.18,
                          max_beta_rad=.72, max_speed_m_s=.2, max_accel_m_s2=.3, clearance_m=.02,
                          sonar_stop_m=.15, encoder_proof_age_s=5., max_xy_variance=.01, max_yaw_variance=.02)
    return cfg


@unittest.skipIf(Rig is None, 'laptop numerical dependencies required')
class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = config()
        self.rig = Rig.from_config(self.cfg)
        self.grid = dict(resolution=.05, width=120,height=100,origin=[-3,-2.5,0],data=[0]*12000)
        self.start = Pose(.8,0,0,0)
        self.goal = [-.70,0,0,0]  # trailer axle; car rear axle ends at -0.20

    def test_geometry_and_signed_offset(self):
        r = self.rig
        self.assertAlmostEqual(r.axle(self.start)[0],.30)
        p = Pose(0,0,0,.1)
        self.assertLess(r.step(p,.01,0).beta,p.beta)
        self.assertGreater(r.step(p,-.01,0).beta,p.beta)
        ds, steer = 1e-6,.25
        next_p = r.step(Pose(0,0,0,0),ds,steer)
        self.assertAlmostEqual(next_p.beta/ds,(1+r.hitch/r.bar)*math.tan(steer)/r.wheelbase,places=4)

    def test_collision_unknown_trailer_only_and_rotated_grid(self):
        self.grid['origin'] = [2.5,-3,math.pi/2]
        # world (.30,0) -> grid (3,.0+2.2) / .05
        self.grid['data'][44*120+60] = -1
        w = World(self.grid,self.rig)
        self.assertLess(w.clearance(self.start),self.rig.margin)
        with self.assertRaisesRegex(ValueError,'start'):
            HybridAStar(w).plan(self.start,self.start)

    def test_jackknife_and_empty_calibration_rejected(self):
        with self.assertRaisesRegex(ValueError,'articulation'):
            HybridAStar(World(self.grid,self.rig)).plan(self.start,self.start._replace(beta=self.rig.beta))
        with self.assertRaisesRegex(ValueError,'calibration'):
            Rig.from_config(load_config(ROOT/'config/vehicle.yaml'))

    def test_straight_reverse_profile_and_tracking(self):
        raw,tracker = calculate(self.cfg,self.grid,self.start,self.goal,.1)
        r, ref = self.rig,tracker.ref
        self.assertTrue(all(k.ds<0 for k in raw[1:]))
        self.assertTrue(r.goal_ok(ref[-1].pose,Pose(-.20,0,0,0)))
        self.assertEqual((ref[0].speed,ref[-1].speed),(0.,0.))
        for a,b in zip(ref,ref[1:]):
            dt = b.t-a.t
            self.assertGreater(dt,0)
            self.assertLessEqual(abs(b.speed-a.speed),r.accel*dt+1e-8)
            self.assertLessEqual(abs(b.steer-a.steer),r.slew*dt+1e-8)
            self.assertLessEqual(abs(b.speed),r.speed)
        sample = next(s for s in ref if s.speed < -.02)
        v,d,err,done = tracker.compute(sample.pose,sample.speed,sample.steer,sample.t)
        self.assertLess(v,0)
        self.assertFalse(done)
        self.assertAlmostEqual(err,0)
        self.assertTrue(tracker.compute(ref[-1].pose,0,ref[-1].steer,ref[-1].t)[-1])
        with self.assertRaisesRegex(ValueError,'deviation'):
            tracker.compute(self.start._replace(y=.5),0,0,0)

    def test_curved_reverse_end_pose(self):
        w = World(self.grid,self.rig)
        goal, _ = w.advance(self.start,-.2,.45)
        raw,tracker = calculate(self.cfg,self.grid,self.start,(*self.rig.axle(goal),goal.beta),0)
        self.assertLess(raw[-1].ds,0)
        self.assertTrue(self.rig.goal_ok(tracker.ref[-1].pose,goal))
        self.assertGreater(abs(tracker.ref[-1].pose.beta),.05)

    def test_forward_correction_stops_before_final_reverse(self):
        raw,tracker=calculate(self.cfg,self.grid,self.start,[.50,0,0,0],0)
        self.assertTrue(any(k.ds>0 for k in raw))
        self.assertLess(raw[-1].ds,0)
        speeds=[point.speed for point in tracker.ref]
        last_forward=max(i for i,v in enumerate(speeds) if v>0)
        first_reverse=next(i for i,v in enumerate(speeds) if v<0)
        self.assertIn(0,speeds[last_forward+1:first_reverse])

    def test_recorded_map_formats_and_axis_flip(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path/'map.pgm').write_bytes(b'P5\n2 2\n255\n'+bytes([0,255,127,255]))
            (path/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.1,
                origin=[-1,-1,0],negate=0,occupied_thresh=.65,free_thresh=.2)))
            grid = read_map(path/'map.yaml')
            self.assertEqual(grid['data'].tolist(),[-1,0,100,0])
            (path/'map.json').write_text(json.dumps(self.grid))
            self.assertEqual(read_map(path/'map.json'),self.grid)

    def test_offline_command_writes_reference_without_server(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            (folder/'vehicle.yaml').write_text(yaml.safe_dump(self.cfg))
            (folder/'grid.json').write_text(json.dumps(self.grid))
            result = subprocess.run([sys.executable,'-m','laptop.parking','offline','--config',str(folder/'vehicle.yaml'),
                '--map',str(folder/'grid.json'),'--start','.8','0','0','0','--goal','-.7','0','0','0',
                '--initial-steer','0','--out',str(folder/'plan.json')],cwd=ROOT,capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            plan = json.loads((folder/'plan.json').read_text())
            self.assertEqual(plan['source'],'offline_plan')
            self.assertLess(min(s[2] for s in plan['reference']),0)

    def test_worker_reports_preview_with_no_motor_feedback(self):
        async def check():
            worker_client=AsyncMock()
            worker_client.hello={'parking_config':self.cfg}
            worker_client.map=dict(self.grid,version=1)
            worker_client.state=dict(server_time=10.,ros_time_s=50.,data=dict(
                vehicle=dict(at=10.,motors=[],trailer=dict(sampled_at=10.,zeroed=True,invalid=0,angle_rad=0)),
                scan_match=dict(x=.8,y=0,yaw=0,stamp_s=49.9)))
            worker=Worker(worker_client)
            job=dict(id='example',map_version=1,goal=self.goal)
            await worker.run_job(job,'planning')
            await worker.plan_task
            await worker.run_job(job,'planning')
            args,packet = worker_client.command.await_args
            self.assertEqual(args,('park_result',))
            self.assertEqual(packet['start'],list(self.start))
            self.assertEqual(packet['job_id'],job['id'])
            self.assertEqual(len(packet['reference'][0]),2)
            worker_client.command.assert_awaited_once()
            worker.clear_job()
        asyncio.run(check())

    def test_tracking_worker_sends_only_common_velocity_command(self):
        async def check():
            client=AsyncMock()
            client.hello=dict(parking_config=self.cfg,session='worker')
            worker=Worker(client)
            _,worker.tracker=calculate(self.cfg,self.grid,self.start,self.goal,0)
            worker.active_id='active'
            client.state=dict(server_time=10.,map_version=1,slam_ready=True,
                map_pose=dict(x=self.start.x,y=0,yaw=0),data=dict(
                    command=dict(owner='worker',mode='AUTO_PARK',armed=True,control='PI',parking=dict(blocked=None)),
                    odometry_status=dict(valid=True),vehicle=dict(at=10.,servo=dict(command_rad=0,pulse_us=1500),
                    trailer=dict(sampled_at=10.,zeroed=True,invalid=0,angle_rad=0),motors=[dict(feedback_valid=True,speed_m_s=0,pwm=0)]*2)))
            job=dict(id='active',map_version=1)
            await worker.run_job(job,'tracking')
            args,values=client.command.await_args
            self.assertEqual(args,('drive',))
            self.assertEqual(set(values),{'speed_m_s','steering_rad','server_time','job_id','error_m'})
            self.assertLess(values['speed_m_s'],0)
            client.state['data']['vehicle']['motors'][0]['feedback_valid']=False
            with self.assertRaisesRegex(ValueError,'속도'):
                await worker.run_job(job,'tracking')
            client.command.assert_awaited_once()
        asyncio.run(check())

    def test_scan_pose_only_allows_planning(self):
        state=dict(server_time=10.,ros_time_s=55.,data=dict(vehicle=dict(at=10.,trailer=dict(
            sampled_at=10.,zeroed=True,invalid=0,angle_rad=.1)),scan_match=dict(stamp_s=54.9,x=1,y=2,yaw=.3)))
        self.assertEqual(measured(state),(Pose(1,2,.3,.1),None,None))
        with self.assertRaisesRegex(ValueError,'map'):
            measured(state,True)
        state['ros_time_s']+=1
        with self.assertRaises(ValueError):
            measured(state)


class ParkingInterlockTests(unittest.TestCase):
    def setUp(self):
        self.cfg=config()
        self.cfg['parking']['commissioned']=True  # artificial state machine inputs, not deployment config
        self.manager=CommandManager(self.cfg)
        self.manager.parking.loc=dict(at=10.,valid=True,version=1,pose=dict(x=0,y=0,yaw=0))
        self.f=dict(at=10.,ready=True,pi_ready=True,epoch=0,motors=[dict(pwm=0,speed_m_s=0,edge_age_s=1.,feedback_valid=True) for _ in range(2)],
                    servo=dict(command_rad=0,pulse_us=1500),trailer=dict(sampled_at=10.,invalid=0,zeroed=True,angle_rad=0),
                    ultrasonic=[dict(at=10.,status='ok',m=1.) for _ in range(4)])
        self.send('claim')
        self.send('park_register','worker')
        self.send('park_plan',goal=[-.8,0,0,0],map_version=1)
        self.job=self.manager.parking.job['id']
        self.send('park_result','worker',job_id=self.job,start=[0,0,0,0],hybrid=[[0,0]],reference=[[0,0]])

    def send(self,kind,sid='browser',**values):
        self.manager.handle(dict(type=kind,session=sid,received=10.,**values),10.,self.f)

    def start(self):
        self.send('park_reset')
        self.send('mode',mode='AUTO_PARK')

    def test_missing_speed_or_uncommissioned_blocks_approved_plan(self):
        self.send('park_reset')
        self.f['motors'][1]['speed_m_s']=None
        with self.assertRaisesRegex(ValueError,'엔코더'):
            self.send('mode',mode='AUTO_PARK')
        self.cfg['parking']['commissioned']=False
        with self.assertRaisesRegex(ValueError,'실차'):
            self.send('mode',mode='AUTO_PARK')
        self.assertFalse(self.manager.armed)

    def test_wrong_start_or_stale_pose_rejected(self):
        self.send('park_reset')
        self.manager.parking.loc['pose']['x']=.3
        with self.assertRaisesRegex(ValueError,'재계획'):
            self.send('mode',mode='AUTO_PARK')
        self.manager.parking.loc['valid']=False
        with self.assertRaisesRegex(ValueError,'Odometry'):
            self.send('mode',mode='AUTO_PARK')

    def test_owner_and_output_interface(self):
        self.start()
        for sid,fields in [('browser',dict(speed_m_s=-.1)),('worker',dict(pwm=[-.1,-.1]))]:
            with self.assertRaises(ValueError):
                self.send('drive',sid,job_id=self.job,steering_rad=0,error_m=0,**fields)
        self.send('drive','worker',job_id=self.job,speed_m_s=-.1,steering_rad=0,error_m=0)
        self.assertLess(self.manager.target['values'][0],0)
        self.send('drive','worker',job_id=self.job,speed_m_s=0,steering_rad=0,error_m=0)
        with self.assertRaisesRegex(ValueError,'final pose'):
            self.send('park_complete','worker',job_id=self.job)
        self.manager.parking.loc['pose']['x']=-.3
        self.send('park_complete','worker',job_id=self.job)
        self.assertEqual(self.manager.parking.phase,'complete')
        self.assertIsNone(self.manager.target)

    def test_loss_never_switches_to_open_loop(self):
        self.start()
        self.f['motors'][0]['feedback_valid']=False
        status=self.manager.tick(10.01,self.f)
        self.assertEqual((status['mode'],status['control'],status['armed'],status['estop']),('MANUAL','PI',False,True))

    def test_heartbeat_cannot_extend_drive_timeout(self):
        self.start()
        self.send('park_ping','worker')
        self.f['at']=self.f['trailer']['sampled_at']=self.manager.parking.loc['at']=10.31
        status=self.manager.tick(10.31,self.f)
        self.assertFalse(status['armed'])
        self.assertIsNone(status['target'])

    def test_map_change_cancels_and_invalidates_result(self):
        self.manager.parking.loc['version']=2
        self.manager.tick(10.,self.f)
        self.assertIsNone(self.manager.parking.job)
        with self.assertRaisesRegex(ValueError,'expired'):
            self.send('park_result','worker',job_id=self.job)

    def test_worker_disconnect_stops(self):
        self.start()
        self.send('release','worker')
        self.assertIsNone(self.manager.owner)
        self.assertTrue(self.manager.estop)
        self.assertIsNone(self.manager.parking.worker)

    def test_old_wheel_edges_cannot_qualify_stationary_start(self):
        self.send('park_reset')
        self.f['motors'][0]['edge_age_s']=100.
        with self.assertRaisesRegex(ValueError,'실제 회전'):
            self.send('mode',mode='AUTO_PARK')

    def test_stale_sonar_interlock(self):
        self.start()
        self.f['ultrasonic'][2]['at']=None
        self.assertFalse(self.manager.tick(10.,self.f)['armed'])
        self.assertIn('초음파',self.manager.parking.detail)

    def test_angle_limit(self):
        self.start()
        self.f['trailer']['angle_rad']=self.cfg['parking']['max_beta_rad']
        self.assertFalse(self.manager.tick(10.,self.f)['armed'])
        self.assertIn('연결각',self.manager.parking.detail)
        self.send('release','worker')

    def test_operator_disconnect_stops_worker(self):
        self.start()
        self.send('release')
        self.assertFalse(self.manager.armed)
        self.assertIsNone(self.manager.owner)
        self.assertEqual(self.manager.parking.worker,'worker')

