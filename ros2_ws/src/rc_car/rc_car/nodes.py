"""Three separate ROS executables: command arbitration, hardware control, odometry."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from .core import (CommandManager, PI, clip, dumps, load_config,
                   odom_configured, integrate)
from .hardware import Hardware

LATEST = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)


class Base(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter('config', '/project/config/vehicle.yaml')
        self.cfg = load_config(self.get_parameter('config').value)

    @staticmethod
    def send(pub, payload):
        pub.publish(String(data=dumps(payload)))


class Manager(Base):
    def __init__(self):
        super().__init__('command_manager')
        self.manager = CommandManager(self.cfg)
        self.feedback = None
        self.pub = self.create_publisher(String, '/rc/command', LATEST)
        self.acks = self.create_publisher(String, '/rc/ack', 20)
        self.actions = self.create_publisher(String, '/rc/action', 5)
        self.paths = self.create_publisher(String, '/rc/parking_paths', LATEST)
        self.create_subscription(String, '/rc/localization_health', self.localization, LATEST)
        self.create_subscription(String, '/rc/request', self.request, 20)
        self.create_subscription(String, '/rc/state', self.state, LATEST)
        self.create_timer(0.02, self.tick)

    def state(self, msg):
        self.feedback = json.loads(msg.data)

    def localization(self, msg):
        self.manager.parking.loc = json.loads(msg.data)

    def fresh_feedback(self):
        if self.feedback and 0 <= time.monotonic() - self.feedback['at'] < self.cfg['hardware']['feedback_timeout_s']:
            return self.feedback
        return None

    def request(self, msg):
        cmd = {}
        try:
            cmd = json.loads(msg.data)
            self.manager.handle(cmd, time.monotonic(), self.fresh_feedback())
            if cmd['type']=='park_result' and self.manager.parking.path_packet:
                self.send(self.paths,self.manager.parking.path_packet)
            if cmd['type'] == 'zero_trailer':
                if self.manager.armed:
                    raise ValueError('disarm before trailer zero')
                self.send(self.actions, cmd)
                return  # hardware confirms the action, not merely its receipt
            ack = {'type': 'ack', 'session': cmd['session'], 'seq': cmd['seq'], 'ok': True}
        except (ValueError, KeyError, TypeError, AssertionError) as exc:
            if cmd.get('session') == self.manager.owner:
                self.manager.stop('rejected_owner_command')
            ack = {'type': 'ack', 'session': cmd.get('session'), 'seq': cmd.get('seq'), 'ok': False, 'error': str(exc)}
        self.send(self.acks, ack)
        self.tick()

    def tick(self):
        self.send(self.pub, self.manager.tick(time.monotonic(), self.fresh_feedback()))


class Controller(Base):
    def __init__(self):
        super().__init__('vehicle_controller')
        self.hw, self.fault = None, None
        try:
            self.hw = Hardware(self.cfg)
        except Exception as exc:
            self.fault = f'hardware_initialization: {exc}'
            self.get_logger().error(self.fault)
        self.command = None
        self.epoch = -1
        self.last = time.monotonic()
        self.steering = None
        self.pwm = [0.0, 0.0]
        self.demand_start = [None, None]
        self.encoder_motion_start = [None, None]
        self.encoder_lost = [False, False]
        self.pi = [PI(m['kp'], m['ki']) for m in self.cfg['hardware']['motors']]
        self.pub = self.create_publisher(String, '/rc/state', LATEST)
        self.trailer_pub = self.create_publisher(Float64, '/trailer/angle', 5)
        self.range_pub = [self.create_publisher(Range, f"/ultrasonic/{s['name']}", 5) for s in self.cfg['ultrasonic']['sensors']]
        self.range_stamps = [None] * 4
        self.acks = self.create_publisher(String, '/rc/ack', 20)
        self.create_subscription(String, '/rc/command', self.accept, LATEST)
        self.create_subscription(String, '/rc/action', self.action, 5)
        self.create_timer(1 / self.cfg['hardware']['control_hz'], self.tick)

    def accept(self, msg):
        try:
            c = json.loads(msg.data)
            if 0 <= time.monotonic() - c['issued'] <= self.cfg['network']['command_timeout_s']:
                self.command = c
        except (ValueError, KeyError, TypeError):
            self.fault = 'invalid_internal_command'

    def action(self, msg):
        c = json.loads(msg.data)
        try:
            if time.monotonic() - c['received'] > self.cfg['network']['max_command_age_s']:
                raise ValueError('expired action')
            if self.command and self.command['armed'] or any(self.pwm):
                raise ValueError('vehicle must be disarmed')
            if not self.hw:
                raise ValueError('hardware unavailable')
            self.hw.zero_trailer()
            result = {'ok': True}
        except Exception as exc:
            result = {'ok': False, 'error': str(exc)}
        self.send(self.acks, dict(type='ack', session=c['session'], seq=c['seq'], **result))

    def tick(self):
        now = time.monotonic()
        dt, self.last = now - self.last, now
        h, c = self.cfg['hardware'], self.command
        stamp = self.get_clock().now().to_msg()
        if not self.hw:
            self.send(self.pub, {'at': now, 'stamp': [stamp.sec, stamp.nanosec], 'ready': False,
                                'pi_ready': False, 'fault': self.fault, 'epoch': self.epoch,
                                'motors': [], 'trailer': None, 'ultrasonic': [], 'servo': None})
            return
        motors, trailer, ranges = self.hw.sample(now)
        ready = h['enabled'] and self.hw.servo is not None
        pi_ready = all(m['calibrated'] and m['observed'] and m['invalid'] <= h['encoder_max_invalid']
                       and spec['pi_verified'] and spec['max_rpm'] and (spec['kp'] > 0 or spec['ki'] > 0)
                       for m, spec in zip(motors, h['motors']))
        active = bool(c and c['armed'] and not c['estop'] and c['target'] and ready)
        if active and c['mode']=='AUTO_PARK' and (self.cfg.get('parking',{}).get('commissioned') is not True or c['control']!='PI'):
            active = False
            self.fault = 'automatic_output_interlock'
        targets = c['target']['values'] if active else [0.0, 0.0]
        try:
            if c and c['armed'] and c['epoch'] > self.epoch:
                self.hw.stop()
                self.epoch = c['epoch']
                self.fault = None
                self.demand_start = [None, None]
                for pi in self.pi:
                    pi.reset()
            if active and (now > c['deadline'] or now - c['issued'] > self.cfg['network']['command_timeout_s']):
                self.fault = 'controller_command_timeout'
            if active and dt > 0.15:
                self.fault = 'control_loop_overrun'
            if active and c['control'] == 'PI' and (not pi_ready or any(self.encoder_lost)):
                self.fault = 'encoder_or_PI_unavailable'
            for i, (target, m) in enumerate(zip(targets, motors)):
                if active and c['control'] == 'PI' and abs(target) >= h['encoder_demand_rpm']:
                    if self.demand_start[i] is None:
                        self.demand_start[i] = now
                    if now - self.demand_start[i] > h['encoder_stall_s'] and (m['edge_age_s'] is None or m['edge_age_s'] > h['encoder_stall_s']):
                        self.fault = f'motor_{i}_encoder_stall'
                    if m['rpm'] is not None and m['rpm'] * target < 0 and abs(m['rpm']) > h['encoder_demand_rpm'] and now - self.demand_start[i] > h['reversal_deadtime_s']:
                        self.fault = f'motor_{i}_encoder_direction'
                else:
                    self.demand_start[i] = None
            if not active or self.fault:
                self.hw.stop()
                self.pwm = [0.0, 0.0]
                for pi in self.pi:
                    pi.reset()
            else:
                angle = clip(c['target']['steering_rad'], min(c['limits']['steering'], self.cfg['servo']['max_angle_rad']))
                if self.steering is None:
                    self.steering = 0.0
                self.steering += clip(angle - self.steering, self.cfg['servo']['slew_rad_s'] * dt)
                self.hw.servo.set(self.steering)
                for i, (target, m, pi) in enumerate(zip(targets, motors, self.pi)):
                    if c['control'] == 'PI':
                        target = clip(target, min(c['limits']['rpm'][i], h['motors'][i]['max_rpm']))
                    limit = min(c['limits']['pwm'], h['max_pwm'], 0.99)
                    self.pwm[i] = pi.step(target, m['rpm'], dt, limit, h['slew_per_s'], now,
                                          h['reversal_deadtime_s'], c['control'] == 'OPEN_LOOP')
                    self.hw.motor(i, self.pwm[i])
        except Exception as exc:
            self.fault = f'output_error: {exc}'
            self.pwm = [0.0, 0.0]
            try:
                self.hw.stop()
            except Exception as stop_error:
                self.get_logger().error(f'Stop write failed, finite PWM lease/physical stop required: {stop_error}')
        for i, m in enumerate(motors):
            if active and abs(targets[i]) > 0:
                if self.encoder_motion_start[i] is None:
                    self.encoder_motion_start[i] = now
            else:
                self.encoder_motion_start[i] = None
            stalled = (self.encoder_motion_start[i] is not None and now-self.encoder_motion_start[i] > h['encoder_stall_s']
                       and (m['edge_age_s'] is None or m['edge_age_s'] > h['encoder_stall_s']))
            if stalled and m['observed']:
                self.encoder_lost[i] = True  # unknown distance interval; restart after repair
            m['feedback_valid'] = bool(m['calibrated'] and m['observed'] and m['invalid'] <= h['encoder_max_invalid'] and not self.encoder_lost[i])
            if not m['feedback_valid']:
                for key in ('rpm','rotor_rpm','speed_m_s','distance_m','path_m','revolutions','wheel_revolutions','direction'):
                    m[key] = None
            m.update(pwm=self.pwm[i], target_rpm=(targets[i] if active and c['control'] == 'PI' else None),
                     target_pwm=(targets[i] if active and c['control'] == 'OPEN_LOOP' else None))
        servo = {'command_rad': self.steering, 'measured_rad': None,
                 'target_rad': c['target']['steering_rad'] if active else None,
                 'pulse_us': self.hw.servo.pulse if self.hw.servo else None,
                 'source': 'calibrated_command_estimate'}
        self.send(self.pub, {'at': now, 'stamp': [stamp.sec, stamp.nanosec], 'ready': bool(ready),
                            'pi_ready': bool(pi_ready and not any(self.encoder_lost)), 'fault': self.fault, 'epoch': self.epoch,
                            'motors': motors, 'servo': servo, 'trailer': trailer,
                            'ultrasonic': ranges, 'sensor_error': self.hw.sensor_error})
        if trailer['angle_rad'] is not None:
            self.trailer_pub.publish(Float64(data=trailer['angle_rad']))
        for i, value in enumerate(ranges):
            if value['at'] is None or value['at'] == self.range_stamps[i]:
                continue
            self.range_stamps[i] = value['at']
            r = Range()
            r.header.stamp = stamp
            r.header.frame_id = value['name']  # mount TF intentionally absent until measured
            r.radiation_type = Range.ULTRASOUND
            r.field_of_view = self.cfg['ultrasonic']['field_of_view_rad']
            r.min_range, r.max_range = self.cfg['ultrasonic']['min_m'], self.cfg['ultrasonic']['max_m']
            r.range = value['m'] if value['m'] is not None else float('nan')
            self.range_pub[i].publish(r)


class OdometryNode(Base):
    def __init__(self):
        super().__init__('encoder_odometry')
        self.pub = self.create_publisher(Odometry, '/odom', 5)
        self.health = self.create_publisher(String, '/rc/odom_status', LATEST)
        self.tf = TransformBroadcaster(self)
        self.previous = None
        self.last_rx = None
        self.broken = False
        self.x = self.y = self.yaw = 0.0
        self.status = {'valid': False, 'reason': 'waiting_for_calibrated_encoders_and_steering'}
        self.create_subscription(String, '/rc/state', self.state, LATEST)
        self.create_timer(0.1, self.tick)

    def tick(self):
        if self.last_rx is None or time.monotonic() - self.last_rx > self.cfg['hardware']['feedback_timeout_s']:
            if self.previous is not None:
                self.broken = True
            self.status = {'valid': False, 'reason': 'feedback_missing_restart_after_recovery'}
        self.send(self.health, dict(self.status, at=time.monotonic()))

    def state(self, msg):
        state = json.loads(msg.data)
        now = time.monotonic()
        if not 0 <= now - state['at'] < self.cfg['hardware']['feedback_timeout_s']:
            return
        self.last_rx = now
        motors, servo = state['motors'], state['servo']
        valid = (odom_configured(self.cfg) and not self.broken and not state['fault'] and len(motors) == 2
                 and servo and servo['command_rad'] is not None and servo['pulse_us'] is not None
                 and all(m['distance_m'] is not None and m['feedback_valid'] for m in motors))
        if not valid:
            if self.previous is not None:
                self.broken = True
            self.status = {'valid': False, 'reason': 'configuration_feedback_or_continuity_invalid'}
            return
        distance = sum(m['distance_m'] for m in motors) / 2
        if self.previous is None:
            self.previous = (distance, state['at'])
            return
        delta, dt = distance - self.previous[0], state['at'] - self.previous[1]
        if dt <= 0 or dt > self.cfg['hardware']['feedback_timeout_s']:
            self.broken = True
            return
        self.previous = (distance, state['at'])
        self.x, self.y, self.yaw = integrate(self.x, self.y, self.yaw, delta, servo['command_rad'], self.cfg['geometry']['wheelbase_m'])
        odom = Odometry()
        odom.header.stamp.sec, odom.header.stamp.nanosec = state['stamp']
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(self.yaw/2), math.cos(self.yaw/2)
        odom.twist.twist.linear.x = delta / dt
        odom.twist.twist.angular.z = delta / dt * math.tan(servo['command_rad']) / self.cfg['geometry']['wheelbase_m']
        for idx, variance in zip((0, 7, 35), self.cfg['geometry']['odom_pose_variance']):
            odom.pose.covariance[idx] = float(variance)
        for idx in (14, 21, 28):
            odom.pose.covariance[idx] = 1e6
            odom.twist.covariance[idx] = 1e6
        odom.twist.covariance[0], odom.twist.covariance[35] = self.cfg['geometry']['odom_twist_variance']
        odom.twist.covariance[7] = 1e3
        self.pub.publish(odom)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = odom.header, 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)
        self.status = {'valid': True, 'reason': 'encoder_odometry_with_commanded_steering_estimate'}


def run(cls):
    rclpy.init()
    node = None
    try:
        node = cls()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            if isinstance(node, Controller) and node.hw:
                node.hw.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def manager_main():
    run(Manager)


def controller_main():
    run(Controller)


def odometry_main():
    run(OdometryNode)
