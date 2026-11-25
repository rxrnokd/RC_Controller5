"""Hardware-independent validation, control and kinematics (SI, output-shaft RPM)."""
import json
import math
import ipaddress
import re

import yaml


def number(value, name='number', low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name}: finite number required')
    if (low is not None and value < low) or (high is not None and value > high):
        raise ValueError(f'{name}: out of range')
    return float(value)


def clip(x, limit):
    return max(-limit, min(limit, x))


def require(condition, message='invalid configuration'):
    if not condition:
        raise ValueError(message)


def load_config(path):
    with open(path, encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)
    h, s, t, u, g, n = (cfg[k] for k in ('hardware', 'servo', 'trailer', 'ultrasonic', 'geometry', 'network'))
    require(g.setdefault('odom_source','wheel') in ('lidar','wheel'), 'geometry.odom_source must be lidar or wheel')
    for section in (h, s, t, u, g, cfg['lidar'], cfg['slam'], *h['motors']):
        for key, value in section.items():
            if key in ('enabled', 'calibrated', 'verified', 'hold_on_stop', 'localization', 'encoder_enabled', 'steering_command_estimate_allowed') or key.endswith('_verified'):
                require(type(value) is bool, f'{key}: YAML boolean required')
    require(len(h['motors']) == 2, 'exactly two motor encoders required')
    pins = [s['gpio'], t['a'], t['b'], t['z']]
    for m in h['motors']:
        require(len(m['pwm']) == 2 and len(m['encoder']) == 2, 'two PWM and two encoder pins required')
        pins += m['pwm'] + m['encoder']
        require(m['direction'] in (-1, 1) and m['encoder_direction'] in (-1, 1))
        number(m['gear_ratio'], 'gear_ratio', 1)
        for k in ('kp', 'ki'):
            number(m[k], k, 0)
        if m['counts_per_motor_rev'] is not None:
            number(m['counts_per_motor_rev'], 'counts_per_motor_rev', 1)
        if m['max_rpm'] is not None:
            number(m['max_rpm'], 'max_rpm', 0.01)
    for sensor in u['sensors']:
        require(re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', sensor['name']), 'invalid ultrasonic name')
        pins += [sensor['trig'], sensor['echo']]
    require(len(set(sensor['name'] for sensor in u['sensors'])) == 4, 'duplicate sensor name')
    require(len(u['sensors']) == 4 and len(set(pins)) == len(pins), 'GPIO conflict')
    require(all(type(p) is int and 0 <= p <= 27 for p in pins), 'BCM GPIO 0..27 only')
    require(s['gpio'] == 13 and s['channel'] == 1, 'this backend requires GPIO13 / PWM channel1')
    require(t['direction'] in (-1, 1))
    number(h['max_pwm'], 'max_pwm', 0, 0.99)
    number(h['pwm_hz'], 'pwm_hz', 10, 10000)
    number(h['control_hz'], 'control_hz', 10, 100)
    number(h['pwm_lease_s'], 'pwm_lease_s', 0.02, 0.15)
    for key in ('slew_per_s', 'feedback_timeout_s', 'encoder_stall_s', 'wheel_diameter_m', 'reversal_deadtime_s'):
        number(h[key], key, 0.001)
    number(s['slew_rad_s'], 'servo.slew_rad_s', 0.001)
    number(n['command_timeout_s'], 'command_timeout_s', 0.05, 0.5)
    number(n['max_command_age_s'], 'max_command_age_s', 0.05, 0.5)
    number(u['timeout_s'], 'ultrasonic.timeout_s', 0.001, 0.1)
    number(u['interval_s'], 'ultrasonic.interval_s', u['timeout_s'] + 0.01)
    number(u['min_m'], 'min_m', 0.001)
    number(u['max_m'], 'max_m', u['min_m'])
    number(u['speed_of_sound_m_s'], 'speed_of_sound', 250, 450)
    number(h['encoder_demand_rpm'], 'encoder_demand_rpm', 0.01)
    number(h['encoder_max_invalid'], 'encoder_max_invalid', 0)
    number(n['port'], 'port', 1024, 65535)
    number(n['state_hz'], 'state_hz', 5, 20)
    number(n['map_period_s'], 'map_period_s', 1, 60)
    number(n['max_map_cells'], 'max_map_cells', 1, 4000000)
    number(n['max_clients'], 'max_clients', 1, 16)
    number(n['max_messages_per_s'], 'max_messages_per_s', 10, 100)
    for cidr in n['allowed_cidrs']:
        ipaddress.ip_network(cidr)
    for variances, count in ((g['odom_pose_variance'], 3), (g['odom_twist_variance'], 2)):
        require(len(variances) == count, 'covariance length')
        for variance in variances:
            number(variance, 'variance', 0.000001)
    if cfg['lidar']['mount_verified']:
        require(len(cfg['lidar']['xyz_rpy']) == 6, 'six lidar extrinsics required')
        for value in cfg['lidar']['xyz_rpy']:
            number(value, 'lidar extrinsic')
    if h['enabled']:
        require(all(h[k] for k in ('wiring_verified', 'motor_power_verified', 'external_stop_verified')), 'power/wiring/physical stop unverified')
        require(h['max_pwm'] > 0, 'max_pwm must be qualified')
    if s['enabled']:
        require(s['calibrated'], 'steering calibration missing')
        number(s['frequency_hz'], 'servo frequency', 40, 400)
        number(s['max_angle_rad'], 'max steering', 0.001, 1.2)
        table = s['calibration']
        require(len(table) >= 3 and any(p[0] == 0 for p in table), 'servo needs centre calibration')
        for angle, pulse in table:
            number(angle, 'calibration angle', -1.2, 1.2)
            number(pulse, 'servo pulse us', 500, min(2500, 1e6 / s['frequency_hz'] - 1))
        require(all(table[i][0] < table[i+1][0] for i in range(len(table)-1)))
        require(all(table[i][1] < table[i+1][1] for i in range(len(table)-1)) or all(table[i][1] > table[i+1][1] for i in range(len(table)-1)))
        require(table[0][0] <= -s['max_angle_rad'] and table[-1][0] >= s['max_angle_rad'])
    if t['enabled']:
        require(t['verified'], 'trailer electrical/ABZ verification required')
        number(t['ppr'], 'trailer PPR', 1, 1024)
        number(t['offset_rad'], 'trailer offset')
        if t['index_angle_rad'] is not None:
            number(t['index_angle_rad'], 'trailer index angle')
    if u['enabled']:
        require(u['electrical_verified'], 'US-100 voltage/jumper must be checked')
    if g['verified']:
        require(g['layout'] == 'rear_axle_left_right')
        number(g['wheelbase_m'], 'wheelbase_m', 0.01)
        number(g['track_m'], 'track_m', 0.01)
    parking = cfg.get('parking', {})
    if parking:
        require(type(parking.get('commissioned')) is bool, 'parking.commissioned must be boolean')
    if parking.get('commissioned'):
        require(g['odom_source']=='wheel','AUTO_PARK drive requires qualified wheel feedback')
        require(odom_configured(cfg) and h['enabled'] and t['enabled'] and u['enabled']
                and all(m['pi_verified'] for m in h['motors'])
                and cfg['lidar']['mount_verified'] and cfg['slam']['enabled'], 'automatic driving calibration incomplete')
        for key in ('drawbar_m','car_front_m','car_back_m','car_width_m','trailer_front_m','trailer_back_m',
                    'trailer_width_m','max_beta_rad','max_speed_m_s','max_accel_m_s2','clearance_m','sonar_stop_m',
                    'encoder_proof_age_s','max_xy_variance','max_yaw_variance','pos_tol_m','yaw_tol_rad','stop_speed_m_s','deviation_m'):
            require(number(parking.get(key),key)>0, f'{key}: positive measurement or limit required')
        number(parking.get('hitch_back_m'),'hitch_back_m',0)
        require(parking['max_beta_rad'] < math.pi/2 and parking['stop_speed_m_s'] < parking['max_speed_m_s'])
    return cfg


def encoder_calibrated(m):
    return bool(m['encoder_enabled'] and m['encoder_verified'] and m['gear_ratio_verified'] and m['counts_per_motor_rev'])


def odom_configured(c):
    if c['geometry']['odom_source']=='lidar':
        return c['lidar']['enabled'] and c['lidar']['mount_verified']
    return bool(c['geometry']['verified'] and c['geometry']['steering_command_estimate_allowed']
                and c['servo']['enabled'] and c['servo']['calibrated']
                and all(encoder_calibrated(m) for m in c['hardware']['motors']))


def lidar_odom_status(ros_now, scan, odom, registration, frame):
    """Qualify scan odometry from sensor timestamps, not web receipt times."""
    for sample in (scan, odom, registration):
        stamp = sample.get('stamp_s', 0)
        if not stamp or not 0 <= ros_now - stamp < 0.5:
            return False, '최근 스캔·라이다 Odometry·정합 상태 대기'
    if scan.get('frame') != frame or not scan.get('has_returns'):
        return False, '라이다 프레임 또는 반사점 확인 필요'
    ratio = registration.get('inliers_ratio')
    if registration['lost'] or ratio is None or not 0 < ratio <= 1:
        return False, '라이다 스캔 정합 실패'
    if odom['frame'] != 'odom' or odom['child_frame'] != 'base_link' or not odom.get('pose_valid'):
        return False, '라이다 Odometry 자세 오류'
    return True, '라이다 스캔 정합'


def dumps(obj):
    return json.dumps(obj, allow_nan=False, separators=(',', ':'))


class Quadrature:
    # One count for every valid A/B edge. Invalid transitions latch as diagnostics.
    TRANSITIONS = {(0, 1): 1, (1, 3): 1, (3, 2): 1, (2, 0): 1,
                   (1, 0): -1, (3, 1): -1, (2, 3): -1, (0, 2): -1}

    def __init__(self, a=0, b=0, direction=1):
        self.state, self.direction = a * 2 + b, direction
        self.count = self.invalid = 0
        self.last_edge = None

    def edge(self, channel, level, now):
        state = (self.state & (1 if channel == 0 else 2)) | (level << (1 if channel == 0 else 0))
        if state != self.state:
            delta = self.TRANSITIONS.get((self.state, state))
            if delta is None:
                self.invalid += 1
            else:
                self.count += delta * self.direction
                self.last_edge = now
            self.state = state
        else:
            # Repeated edge level indicates at least one unobserved transition.
            self.invalid += 1


class PI:
    def __init__(self, kp, ki):
        self.kp, self.ki, self.integral, self.output = kp, ki, 0.0, 0.0
        self.last_sign, self.zero_since = 0, None

    def reset(self):
        self.integral = self.output = 0.0

    def step(self, target, measured, dt, limit, slew, now, deadtime, open_loop=False):
        if target == 0:
            self.reset()
            if self.zero_since is None:
                self.zero_since = now
            return 0.0
        sign = 1 if target > 0 else -1
        if self.last_sign and sign != self.last_sign:
            self.reset()
            if self.zero_since is None:
                self.zero_since = now
            if now - self.zero_since < deadtime:
                return 0.0
        self.last_sign, self.zero_since = sign, None
        error = 0.0 if open_loop else target - number(measured,'measured RPM')
        desired = target if open_loop else self.kp * error + self.integral + self.ki * error * dt
        # Avoid active reverse braking from a PI overspeed error.
        desired = max(0.0, desired) if sign > 0 else min(0.0, desired)
        bounded = clip(desired, limit)
        output = self.output + clip(bounded - self.output, slew * dt)
        output = clip(output, limit)  # lowering a live limit takes effect immediately
        if not open_loop and (abs(desired - output) < 1e-12 or error * (desired - output) <= 0):
            self.integral = clip(self.integral + self.ki * error * dt, limit)
        self.output = output
        return output


def servo_pulse(table, angle):
    for (a, p), (b, q) in zip(table, table[1:]):
        if a <= angle <= b:
            return p + (q - p) * (angle - a) / (b - a)
    raise ValueError('steering outside calibrated range')


def wheel_targets(speed, steering, cfg):
    if not cfg['geometry']['verified']:
        raise ValueError('geometry unverified')
    g = cfg['geometry']
    curvature = math.tan(steering) / g['wheelbase_m']
    require(abs(curvature * g['track_m'] / 2) < 1, 'turn radius is inside the rear axle')
    rpm = speed * 60 / (math.pi * cfg['hardware']['wheel_diameter_m'])
    return [number(rpm * (1 - curvature * g['track_m'] / 2), 'left RPM'),
            number(rpm * (1 + curvature * g['track_m'] / 2), 'right RPM')]


def integrate(x, y, yaw, distance, steering, wheelbase):
    dyaw = distance * math.tan(steering) / wheelbase
    scale = 1.0 if abs(dyaw) < 1e-10 else math.sin(dyaw / 2) / (dyaw / 2)
    return (x + distance * scale * math.cos(yaw + dyaw / 2),
            y + distance * scale * math.sin(yaw + dyaw / 2),
            math.atan2(math.sin(yaw + dyaw), math.cos(yaw + dyaw)))


class CommandManager:
    """Only accepted, fresh owner drive commands renew the motor lease."""
    def __init__(self, cfg):
        from .parking_control import Parking
        self.parking = Parking()
        self.cfg, self.owner = cfg, None
        self.estop, self.armed, self.mode, self.control = True, False, 'MANUAL', 'OPEN_LOOP'
        self.target = None
        self.deadline = 0.0
        self.epoch = 0
        self.limits = {'pwm': cfg['hardware']['max_pwm'], 'rpm': [m['max_rpm'] or 0 for m in cfg['hardware']['motors']],
                       'steering': cfg['servo']['max_angle_rad'] or 0}
        self.reason = 'startup_estop'

    def stop(self, reason, estop=False):
        self.armed, self.target, self.reason = False, None, reason
        self.estop |= estop

    def handle(self, cmd, now, feedback):
        kind, session = cmd['type'], cmd['session']
        if kind not in ('estop', 'stop'):
            number(cmd['received'], 'received', now - self.cfg['network']['max_command_age_s'], now + 0.01)
        if self.parking.handle(self, cmd, now, feedback):
            return
        if kind in ('estop', 'stop'):
            self.stop(kind, kind == 'estop')
            return
        if kind == 'claim':
            if self.owner not in (None, session):
                raise ValueError('another client owns control')
            self.owner = session
            self.stop('owner_claimed')
            return
        if session != self.owner:
            raise ValueError('claim control first')
        if kind == 'release':
            self.stop('owner_released')
            self.owner = None
        elif kind == 'mode':
            self.stop('mode_change')
            if cmd['mode'] != 'MANUAL':
                raise ValueError('unknown mode')
            self.mode = 'MANUAL'
        elif kind == 'control':
            self.stop('control_change')
            if cmd['control'] not in ('PI', 'OPEN_LOOP'):
                raise ValueError('unknown control mode')
            self.control = cmd['control']
        elif kind == 'limits':
            self.stop('limits_changed')
            limits = {'pwm': number(cmd['pwm'], 'pwm', 0, self.cfg['hardware']['max_pwm']),
                      'steering': number(cmd['steering'], 'steering', 0, self.cfg['servo']['max_angle_rad'] or 0)}
            require(len(cmd['rpm']) == 2, 'two RPM limits required')
            limits['rpm'] = [number(v, 'rpm limit', 0, m['max_rpm'] or 0) for v, m in zip(cmd['rpm'], self.cfg['hardware']['motors'])]
            self.limits = limits
        elif kind == 'arm':
            if not feedback or not feedback.get('ready'):
                raise ValueError('fresh hardware readiness required')
            if any(abs(m.get('pwm', 0)) > 1e-9 for m in feedback['motors']):
                raise ValueError('outputs must be zero before arm')
            if self.control == 'PI' and not feedback.get('pi_ready'):
                raise ValueError('PI calibration/encoder feedback unavailable')
            self.estop, self.armed, self.reason = False, True, 'armed_waiting_command'
            self.epoch += 1
            self.target = None
            self.deadline = now + self.cfg['network']['command_timeout_s']
        elif kind == 'drive':
            if not self.armed or self.estop:
                raise ValueError('explicit arm required')
            steering = clip(number(cmd['steering_rad'], 'steering_rad'), self.limits['steering'])
            if self.control == 'PI':
                if 'speed_m_s' in cmd:
                    targets = wheel_targets(number(cmd['speed_m_s'], 'speed_m_s'), steering, self.cfg)
                else:
                    require(len(cmd['rpm']) == 2, 'two RPM targets required')
                    targets = [number(v, 'rpm') for v in cmd['rpm']]
                # Preserve wheel speed ratio when clipping an Ackermann command.
                factor = min([1.0] + [limit / abs(v) for v, limit in zip(targets, self.limits['rpm']) if v])
                values = [v * factor for v in targets]
            else:
                require(len(cmd['pwm']) == 2, 'two PWM targets required')
                values = [clip(number(v, 'pwm'), self.limits['pwm']) for v in cmd['pwm']]
            self.target = {'values': values, 'steering_rad': steering}
            self.deadline = now + self.cfg['network']['command_timeout_s']
            self.reason = 'command_active'
        elif kind != 'zero_trailer':
            raise ValueError('unknown command')

    def tick(self, now, feedback):
        self.parking.watch(self, now, feedback)
        if self.armed and (now > self.deadline or feedback is None):
            self.stop('command_or_feedback_timeout')
        if self.armed and feedback and feedback.get('fault'):
            # A prior controller latch can be cleared only by a NEW arm epoch.
            if feedback.get('epoch', -1) >= self.epoch:
                self.stop('controller_fault', True)
        parking = self.parking.status(self, now, feedback)
        return {'mode': self.mode, 'control': self.control, 'owner': self.owner,
                'estop': self.estop, 'armed': self.armed, 'reason': self.reason,
                'epoch': self.epoch, 'deadline': self.deadline, 'issued': now,
                'limits': self.limits, 'target': self.target if self.armed else None,
                'parking': parking, 'auto_available': parking['online'] and parking['phase']=='planned' and parking['blocked'] is None}
