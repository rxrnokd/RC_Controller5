"""Run from the project root: python -m laptop.parking --help."""
import argparse
import asyncio
import json
import math
from pathlib import Path
import time
from threading import Event

import numpy as np
import yaml

from .planner import HybridAStar, Pose, Rig, World, wrap
from .mpc import PlanningMPC, TrackingMPC


def read_map(path):
    if path.suffix.lower()=='.json':
        return json.loads(path.read_text(encoding='utf-8'))
    from PIL import Image
    metadata = yaml.safe_load(path.read_text(encoding='utf-8'))
    if metadata.get('mode','trinary')!='trinary':
        raise ValueError('expected Nav2 trinary map')
    with Image.open(path.parent/metadata['image']) as image:
        if image.mode != 'L':
            raise ValueError('expected grayscale occupancy image (PGM)')
        intensity = np.asarray(image,dtype=float)[::-1]/255.
    probability = intensity if metadata.get('negate') else 1-intensity
    grid = np.where(probability < metadata['free_thresh'],0,np.where(probability > metadata['occupied_thresh'],100,-1))
    return dict(width=grid.shape[1],height=grid.shape[0],resolution=metadata['resolution'],
                origin=metadata['origin'],frame='map',data=grid.ravel())


def calculate(config, grid, start, requested_goal, initial_steer, cancel=None):
    rig = Rig.from_config(config)
    x,y,phi,beta = requested_goal
    yaw = wrap(phi+beta)
    goal = Pose(x+rig.bar*math.cos(phi)+rig.hitch*math.cos(yaw),
                y+rig.bar*math.sin(phi)+rig.hitch*math.sin(yaw),yaw,beta)
    if not math.isfinite(initial_steer) or abs(initial_steer)>rig.steer:
        raise ValueError('initial steering outside calibrated range')
    world = World(grid,rig,cancel)
    p = config['parking']
    raw = HybridAStar(world,p['max_nodes'],p['search_seconds']).plan(start,goal)
    raw[0] = raw[0]._replace(steer=initial_steer)
    profile = PlanningMPC(world).optimize(raw,goal)
    if not rig.goal_ok(profile[-1].pose,goal):
        raise ValueError('reference misses final car/trailer pose')
    return raw,TrackingMPC(world,profile,goal)


def measured(state, tracking=False):
    data = state['data']
    if state.get('map_error'):
        raise ValueError(str(state['map_error']))
    vehicle = data.get('vehicle') or {}
    if not 0 <= state['server_time']-vehicle.get('at',-math.inf)<0.3:
        raise ValueError('차량 상태 만료')
    trailer = vehicle.get('trailer') or {}
    if (not trailer.get('zeroed') or trailer.get('invalid') != 0 or trailer.get('angle_rad') is None
            or not 0 <= state['server_time']-trailer.get('sampled_at',-math.inf)<0.3):
        raise ValueError('MT6701 각도/영점 필요')
    pose = state.get('map_pose') if state.get('slam_ready') else None
    if not pose and not tracking:
        match = data.get('scan_match',{})
        if 0 <= state['ros_time_s']-match.get('stamp_s',-math.inf)<0.5:
            pose = match
    if not pose:
        raise ValueError('최근 map 자세 필요 · 저장 지도는 offline 명령 사용')
    measured_pose = Pose(pose['x'],pose['y'],pose['yaw'],trailer['angle_rad'])
    servo = vehicle.get('servo') or {}
    steer = servo.get('command_rad') if servo.get('pulse_us') is not None else None
    if not tracking:
        return measured_pose, None, steer
    motors = vehicle.get('motors',[])
    if len(motors)!=2 or not all(m.get('feedback_valid') and m.get('speed_m_s') is not None for m in motors):
        raise ValueError('측정 속도 없음')
    if steer is None or not data.get('odometry_status',{}).get('valid'):
        raise ValueError('조향/위치 피드백 없음')
    return measured_pose,sum(m['speed_m_s'] for m in motors)/2,steer


def preview(seq, rig):
    indices=np.linspace(0,len(seq)-1,min(len(seq),512),dtype=int)
    return [[round(value,4) for value in rig.axle(seq[i].pose)[:2]] for i in indices]


class Worker:
    def __init__(self, client):
        self.client=client
        self.config=client.hello.get('parking_config',{})
        if not self.config.get('parking'):
            raise ValueError('Pi의 주차 지원 ROS 패키지와 vehicle.yaml의 parking 설정을 확인하세요.')
        self.active_id=None
        self.plan_task=None
        self.tracker=None
        self.started=None
        self.cancel=None
        self.last_sample=None

    def clear_job(self):
        if self.cancel:
            self.cancel.set()
        if self.plan_task:
            self.plan_task.cancel()
            if self.plan_task.done() and not self.plan_task.cancelled():
                self.plan_task.exception()
        self.plan_task=self.tracker=self.started=self.active_id=self.last_sample=None

    async def heartbeat(self):
        while True:
            await self.client.command('park_ping')
            await asyncio.sleep(0.4)

    async def run_job(self, job, phase):
        c=self.client
        job_id=job['id']
        if phase=='planning' and job_id!=self.active_id:
            self.clear_job()
            self.active_id=job_id
            pose,_,steer=measured(c.state)
            if not c.map or c.map['version']!=job['map_version']:
                raise ValueError('최신 지도를 수신한 뒤 재계획')
            # A preview may assume stationary centred steering. Tracking always needs feedback.
            self.cancel=Event()
            self.plan_task=asyncio.create_task(asyncio.to_thread(calculate,self.config,c.map,pose,job['goal'],
                                                                0. if steer is None else steer,self.cancel))
        if phase=='planning' and self.plan_task and self.plan_task.done():
            task,self.plan_task=self.plan_task,None
            raw,self.tracker=task.result()
            rig=self.tracker.w.r
            await c.command('park_result',job_id=job_id,hybrid=preview(raw,rig),
                            reference=preview(self.tracker.ref,rig),start=list(raw[0].pose))
        if phase!='tracking':
            return
        if job_id!=self.active_id or self.tracker is None:
            raise ValueError('reference unavailable')
        state=c.state
        cmd=state['data']['command']
        if cmd['owner']!=c.hello['session'] or cmd['mode']!='AUTO_PARK' or not cmd['armed'] or cmd['control']!='PI':
            raise ValueError('자동 제어권 없음')
        if state['map_version']!=job['map_version'] or cmd['parking']['blocked']:
            raise ValueError(cmd['parking']['blocked'] or '지도 변경')
        pose,speed,steer=measured(state,True)
        now=time.monotonic()
        sample=state['server_time']
        if sample==self.last_sample:
            return
        dt=0.1 if self.last_sample is None else sample-self.last_sample
        self.last_sample=sample
        if self.started is None:
            self.started=now
        velocity,angle,error,complete=await asyncio.to_thread(self.tracker.compute,pose,speed,steer,now-self.started,dt)
        if time.monotonic()-now > 0.12 or c.state['server_time']-state['server_time']>0.15:
            raise ValueError('계산 기한 초과')
        stopped=all(m['pwm']==0 for m in state['data']['vehicle']['motors'])
        if complete and stopped and (cmd.get('target') or {}).get('values')==[0.,0.]:
            await c.command('park_complete',job_id=job_id)
        else:
            await c.command('drive',job_id=job_id,server_time=sample,speed_m_s=velocity,
                            steering_rad=steer if complete else angle,error_m=error)

    async def run(self):
        await self.client.command('park_register')
        print('노트북 주차 알고리즘 연결됨. 웹에서 제어권 획득 후 경로 계획을 선택하세요.',flush=True)
        ping=asyncio.create_task(self.heartbeat())
        try:
            while not self.client.reader.done():
                if ping.done():
                    ping.result()
                start=time.monotonic()
                status=self.client.state['data'].get('command',{}).get('parking',{})
                job=status.get('job')
                if job:
                    try:
                        await self.run_job(job,status['phase'])
                    except Exception as exc:
                        print(str(exc),flush=True)
                        try:
                            await self.client.command('park_abort',job_id=job['id'],reason=str(exc))
                        except RuntimeError as rejected:
                            if str(rejected)!='expired parking job':
                                raise
                elif self.active_id is not None:
                    self.clear_job()
                await asyncio.sleep(max(.01,.1-(time.monotonic()-start)))
        finally:
            self.clear_job()
            ping.cancel()
            await asyncio.gather(ping,return_exceptions=True)


async def online(url):
    from laptop.client import VehicleClient
    async with VehicleClient(url) as client:
        await Worker(client).run()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    modes=parser.add_subparsers(dest='mode',required=True)
    modes.add_parser('connect').add_argument('url')
    offline=modes.add_parser('offline',help='plan on a recorded occupancy grid; no connection or motor output')
    offline.add_argument('--config',default='config/vehicle.yaml',type=Path)
    offline.add_argument('--map',required=True,type=Path)
    offline.add_argument('--start',nargs=4,required=True,type=float,metavar=('CAR_X','CAR_Y','CAR_YAW','BETA'))
    offline.add_argument('--goal',nargs=4,required=True,type=float,metavar=('TRAILER_X','TRAILER_Y','TRAILER_YAW','BETA'))
    offline.add_argument('--initial-steer',type=float,required=True)
    offline.add_argument('--out',required=True,type=Path)
    args=parser.parse_args()
    try:
        if args.mode=='connect':
            asyncio.run(online(args.url))
        else:
            cfg=yaml.safe_load(args.config.read_text(encoding='utf-8'))
            raw,tracker=calculate(cfg,read_map(args.map),Pose(*args.start),args.goal,args.initial_steer)
            args.out.write_text(json.dumps(dict(frame='map',source='offline_plan',hybrid=raw,reference=tracker.ref),
                                           allow_nan=False),encoding='utf-8')
            print(f'{len(raw)} Hybrid A* poses, {len(tracker.ref)} timed references: {args.out}')
    except (ValueError,KeyError,RuntimeError,OSError,ImportError) as exc:
        parser.exit(1,f'Parking unavailable: {exc}\n')


if __name__=='__main__':
    main()

