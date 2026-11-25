"""Vehicle-side parking session arbitration. No search or optimization on the Pi."""
import math
import uuid
from .core import number, require, wheel_targets


def finite(value):
    return type(value) in (float, int) and math.isfinite(value)


def fresh(stamp, now, timeout):
    return finite(stamp) and 0 <= now-stamp < timeout


def pose_matches(p, loc, f, target, trailer_goal=False):
    pose = loc.get('pose') or {}
    x, y, yaw = (pose.get(k) for k in ('x', 'y', 'yaw'))
    beta = f['trailer']['angle_rad']
    if not all(map(finite, (x,y,yaw,beta))):
        return False
    def axle(cx, cy, heading, phi):
        return (cx-p['hitch_back_m']*math.cos(heading)-p['drawbar_m']*math.cos(phi),
                cy-p['hitch_back_m']*math.sin(heading)-p['drawbar_m']*math.sin(phi))
    tx, ty, angle, tb = target
    tyaw = angle+tb if trailer_goal else angle
    tphi = tyaw-tb
    if trailer_goal:
        # Invert the hitch transform to compare both rigid bodies at the requested goal.
        dx,dy = axle(0,0,tyaw,tphi)
        tx,ty = tx-dx,ty-dy
    def angular_error(a):
        return abs(math.atan2(math.sin(a),math.cos(a)))
    return (math.dist((x,y),(tx,ty)) <= p['pos_tol_m']
            and math.dist(axle(x,y,yaw,yaw-beta),axle(tx,ty,tyaw,tphi)) <= p['pos_tol_m']
            and max(angular_error(yaw-tyaw),angular_error(yaw-beta-tphi)) <= p['yaw_tol_rad'])


def feedback_problem(cfg, f, loc, now, moving=False):
    p = cfg.get('parking', {})
    if cfg['geometry']['odom_source']=='lidar':
        return '라이다 위치 추정 · 자동주차는 경로 계획만 가능'
    if p.get('commissioned') is not True:
        return '실차 검증 잠금'
    timeout = cfg['network']['command_timeout_s']
    if not f or not fresh(f.get('at'),now,timeout) or f.get('fault') or not f.get('ready'):
        return '차량 피드백 또는 하드웨어 오류'
    motors = f.get('motors', [])
    if len(motors)!=2 or not f.get('pi_ready') or not all(m.get('feedback_valid') and finite(m.get('speed_m_s')) for m in motors):
        return '양쪽 엔코더 속도 피드백 필요'
    if not loc or not loc.get('valid') or not fresh(loc.get('at'),now,timeout):
        return '최근 지도 위치·스캔·Odometry 필요'
    t = f.get('trailer') or {}
    if not t.get('zeroed') or t.get('invalid') != 0 or not finite(t.get('angle_rad')) or not fresh(t.get('sampled_at'),now,timeout):
        return '트레일러 영점·상대각 필요'
    if not p.get('max_beta_rad') or abs(t['angle_rad']) >= p['max_beta_rad']:
        return '연결각 한계'
    if not f.get('servo') or not finite(f['servo'].get('pulse_us')) or not finite(f['servo'].get('command_rad')):
        return '조향 보정 상태 필요'
    if not p.get('sonar_stop_m') or len(f.get('ultrasonic', [])) != 4 or not all(
        s.get('status')=='ok' and fresh(s.get('at'),now,cfg['hardware']['feedback_timeout_s'])
        and finite(s.get('m')) and s['m'] > p['sonar_stop_m'] for s in f['ultrasonic']):
        return '초음파 장애물 또는 피드백 오류'
    if not moving and any(abs(m['speed_m_s']) > p['stop_speed_m_s'] or m.get('pwm') != 0 for m in motors):
        return '차량 정지 필요'
    return None


class Parking:
    def __init__(self):
        self.worker = self.operator = None
        self.seen = -math.inf
        self.loc = None
        self.job = None
        self.phase = 'idle'
        self.detail = ''
        self.error_m = None
        self.path_packet = None

    def status(self, m, now, f):
        return dict(worker=self.worker, online=self.worker is not None and now-self.seen < 1.5,
                    job=self.job, phase=self.phase, detail=self.detail, error_m=self.error_m,
                    blocked=feedback_problem(m.cfg,f,self.loc,now,m.mode=='AUTO_PARK'))

    def end(self, m, reason, phase='stopped'):
        if m.mode == 'AUTO_PARK':
            m.mode, m.owner = 'MANUAL', None
            m.stop(reason, phase!='complete')
        self.phase, self.detail = phase, reason
        self.job = self.path_packet = self.operator = None
        self.error_m = None

    def watch(self, m, now, f):
        if self.worker and now-self.seen > 1.5:
            if self.job:
                self.end(m,'노트북 연결 만료')
            self.worker = None
        if self.job and self.loc and self.job['map_version'] != self.loc['version']:
            self.end(m,'지도 변경 · 정지 후 재계획')
        if m.mode=='AUTO_PARK':
            reason = feedback_problem(m.cfg,f,self.loc,now,True)
            if reason or not m.armed or now > m.deadline:
                self.end(m, reason or '명령 만료')

    def handle(self, m, c, now, f):
        kind, sid = c['type'], c['session']
        if kind in ('stop','estop'):
            if self.job:
                self.end(m,kind)
            return False
        if kind=='release' and sid in (self.worker,self.operator):
            if self.job:
                self.end(m,'주차 연결 종료')
            if sid==self.worker:
                self.worker=None
            return sid!=m.owner
        if self.job and m.mode=='MANUAL' and sid==self.operator and kind in ('arm','zero_trailer','limits','control'):
            self.end(m,'차량 상태 변경 · 재계획 필요')
        if kind=='mode' and c.get('mode')=='MANUAL' and m.mode=='AUTO_PARK' and sid==self.operator:
            self.end(m,'수동 모드 전환')
            return True
        if kind=='park_register':
            require(sid!=m.owner and self.worker in (None,sid), 'algorithm session unavailable')
            self.worker,self.seen=sid,now
            return True
        if sid==self.worker:
            require(kind in ('park_ping','park_result','park_progress','park_complete','park_abort','drive'), 'algorithm session is restricted')
            if kind=='park_ping':
                self.seen=now
                return True
            require(self.job and c.get('job_id')==self.job['id'], 'expired parking job')
            self.seen=now
            if kind=='park_abort':
                self.end(m,str(c.get('reason','노트북 정지'))[:240],'failed')
                return True
            if kind=='park_result':
                require(self.phase=='planning','unexpected result')
                if c.get('error'):
                    self.end(m,str(c['error'])[:240],'failed')
                else:
                    start = c['start']
                    require(isinstance(start,list) and len(start)==4,'invalid plan start pose')
                    paths = {}
                    for key in ('hybrid','reference'):
                        points=c[key]
                        require(isinstance(points,list) and 1 <= len(points)<=512,'invalid preview path')
                        paths[key]=[[number(x) for x in point] for point in points if isinstance(point,list) and len(point)==2]
                        require(len(paths[key])==len(points),'invalid preview point')
                    self.path_packet=dict(type='parking_paths',job_id=self.job['id'],**paths)
                    self.job['start']=[number(x) for x in start]
                    self.phase='planned'
                return True
            require(m.mode=='AUTO_PARK' and m.owner==sid and self.phase=='tracking','no automatic control authority')
            if kind=='park_complete':
                require(not feedback_problem(m.cfg,f,self.loc,now,False),'completion requires measured standstill')
                require(m.target and m.target['values']==[0.,0.],'zero speed command required')
                require(pose_matches(m.cfg['parking'],self.loc,f,self.job['goal'],True),'final pose outside tolerance')
                self.end(m,'complete','complete')
                return True
            if kind=='park_progress':
                self.error_m=number(c['error_m'],'error',0)
                return True
            require(m.control=='PI' and 'speed_m_s' in c and 'pwm' not in c and 'rpm' not in c,'PI velocity required')
            require(now<=m.deadline and self.loc and self.loc['version']==self.job['map_version'],'expired auto command or map')
            reason=feedback_problem(m.cfg,f,self.loc,now,True)
            require(not reason,reason)
            v=number(c['speed_m_s'],'auto speed',-m.cfg['parking']['max_speed_m_s'],m.cfg['parking']['max_speed_m_s'])
            steer=number(c['steering_rad'],'steering',-m.limits['steering'],m.limits['steering'])
            require(all(abs(rpm)<=limit for rpm,limit in zip(wheel_targets(v,steer,m.cfg),m.limits['rpm'])),'wheel RPM limit: replan')
            self.error_m=number(c['error_m'],'tracking error',0,m.cfg['parking']['deviation_m'])
            return False
        if kind=='park_reset':
            require(sid==m.owner and not m.armed and m.mode=='MANUAL','claim and stop first')
            m.estop=False
            return True
        if kind=='park_plan':
            require(sid==m.owner and not m.armed and m.mode=='MANUAL','claim and stop before planning')
            require(self.worker and now-self.seen < 1.5,'노트북 알고리즘 미접속')
            goal=c['goal']
            require(isinstance(goal,list) and len(goal)==4,'goal requires trailer x,y,yaw and final articulation')
            require(self.phase!='planning','계획 중입니다. 취소 후 다시 요청하세요.')
            require(self.loc and c['map_version']==self.loc['version'] and self.loc['version']>0,'map changed/unavailable')
            self.operator=sid
            self.job=dict(id=uuid.uuid4().hex,goal=[number(x) for x in goal],map_version=c['map_version'])
            self.phase,self.detail,self.error_m,self.path_packet='planning','',None,None
            return True
        if kind=='mode' and c.get('mode')=='AUTO_PARK':
            require(sid==m.owner,'claim before switching mode')
            if m.armed:
                m.stop('mode_change')
                raise ValueError('차량 정지 후 다시 요청')
            require(not m.estop,'비상정지 해제 필요')
            reason=feedback_problem(m.cfg,f,self.loc,now)
            require(not reason,reason)
            require(self.phase=='planned' and self.operator==sid and self.worker and now-self.seen<1.5,'validated plan required')
            require(self.job['map_version']==self.loc['version'],'map changed: replan')
            max_age=m.cfg['parking']['encoder_proof_age_s']
            require(all(finite(w.get('edge_age_s')) and 0 <= w['edge_age_s']+now-f['at'] <= max_age for w in f['motors']),
                    '양쪽 엔코더의 최근 실제 회전 확인 필요')
            require(pose_matches(m.cfg['parking'],self.loc,f,self.job['start']),'계획 시작 자세 변경 · 재계획 필요')
            require(m.limits['pwm']>0 and min(m.limits['rpm'])>0 and m.limits['steering']>0,'PI output limits required')
            m.owner,m.mode,m.control,m.armed=self.worker,'AUTO_PARK','PI',True
            m.target=None
            m.epoch+=1
            m.deadline=now+m.cfg['network']['command_timeout_s']
            m.reason,self.phase='auto_start','tracking'
            return True
        return False

