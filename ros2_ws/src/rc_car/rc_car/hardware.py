"""Pi-only lgpio edge capture/motor PWM and kernel PWM steering. No mock backend."""
import math
import threading
import time
from pathlib import Path

from .core import Quadrature, encoder_calibrated, servo_pulse


class KernelServo:
    def __init__(self, cfg):
        self.cfg = cfg
        chip = Path(cfg['pwmchip'])
        self.path = chip / f"pwm{cfg['channel']}"
        if not self.path.exists():
            (chip / 'export').write_text(str(cfg['channel']))
            for _ in range(50):
                if self.path.exists():
                    break
                time.sleep(0.01)
        self.write('enable', 0)
        self.write('duty_cycle', 0)
        self.write('period', round(1e9 / cfg['frequency_hz']))
        (self.path / 'polarity').write_text('normal')
        self.pulse = None

    def write(self, name, value):
        (self.path / name).write_text(str(int(value)))

    def set(self, angle):
        pulse = servo_pulse(self.cfg['calibration'], angle)
        self.write('duty_cycle', round(pulse * 1000))
        self.write('enable', 1)
        self.pulse = pulse

    def disable(self):
        self.write('enable', 0)
        self.pulse = None


class Hardware:
    def __init__(self, cfg):
        self.cfg = cfg
        self.lock = threading.RLock()
        self.callbacks, self.claimed = [], []
        self.encoders = [None, None]
        self.trailer = None
        self.zero_count = None
        self.zero_angle = 0.0
        self.index_count = 0
        self.index_pending = None
        self.ranges = [{'name': s['name'], 'm': None, 'status': 'disabled', 'at': None} for s in cfg['ultrasonic']['sensors']]
        self.echo_rise, self.echo_fall = None, None
        self.echo_active = None
        self.echo_done = threading.Event()
        self.shutdown = threading.Event()
        self.sensor_error = None
        self.sensor_thread = None
        self.servo, self.handle, self.lg = None, None, None
        self.last_counts = [0, 0]
        self.last_measure = time.monotonic()
        self.previous_pwm = [0.0, 0.0]
        self.burst_until = [0.0, 0.0]
        self.total_path = [0.0, 0.0]
        try:
            needed = cfg['hardware']['enabled'] or any(m['encoder_enabled'] for m in cfg['hardware']['motors']) or cfg['trailer']['enabled'] or cfg['ultrasonic']['enabled']
            if needed:
                import lgpio
                self.lg = lgpio
                self.handle = lgpio.gpiochip_open(cfg['hardware']['gpiochip'])
                for m in cfg['hardware']['motors']:
                    if cfg['hardware']['enabled']:
                        for pin in m['pwm']:
                            self.output(pin)
                for i, m in enumerate(cfg['hardware']['motors']):
                    if m['encoder_enabled']:
                        self.encoders[i] = self.quadrature(*m['encoder'], m['encoder_direction'])
                t = cfg['trailer']
                if t['enabled']:
                    self.trailer = self.quadrature(t['a'], t['b'], t['direction'])
                    self.alert(t['z'], self.index)
                if cfg['ultrasonic']['enabled']:
                    for i, s in enumerate(cfg['ultrasonic']['sensors']):
                        self.output(s['trig'])
                        self.alert(s['echo'], lambda chip, gpio, level, tick, idx=i: self.echo(idx, level, tick))
                    self.sensor_thread = threading.Thread(target=self.ultrasound, daemon=True)
                    self.sensor_thread.start()
            if cfg['servo']['enabled']:
                self.servo = KernelServo(cfg['servo'])
                # No pulse until explicit arm/command, even with a calibrated configuration.
        except Exception:
            self.close()
            raise

    def output(self, pin):
        self.lg.gpio_claim_output(self.handle, pin, 0)
        self.claimed.append(pin)

    def alert(self, pin, callback):
        self.lg.gpio_claim_alert(self.handle, pin, self.lg.BOTH_EDGES)
        self.claimed.append(pin)
        self.callbacks.append(self.lg.callback(self.handle, pin, self.lg.BOTH_EDGES, callback))

    def quadrature(self, a, b, direction):
        q = Quadrature(direction=direction)
        # Register each level event; do NOT read the other pin later in the callback.
        def edge(channel, level):
            if level in (0, 1):
                with self.lock:
                    q.edge(channel, level, time.monotonic())
        self.alert(a, lambda chip, gpio, level, tick: edge(0, level))
        self.alert(b, lambda chip, gpio, level, tick: edge(1, level))
        with self.lock:
            q.state = self.lg.gpio_read(self.handle, a) * 2 + self.lg.gpio_read(self.handle, b)
            q.count, q.invalid, q.last_edge = 0, 0, None
        return q

    def index(self, chip, gpio, level, tick):
        if level == 1:
            with self.lock:
                self.index_count += 1
                self.index_pending = self.trailer.count

    def zero_trailer(self):
        with self.lock:
            if self.trailer is None or self.trailer.invalid:
                raise ValueError('trailer ABZ unavailable or invalid')
            self.zero_count = self.trailer.count
            self.zero_angle = 0.0
            self.index_pending = None

    def echo(self, index, level, tick):
        with self.lock:
            if self.echo_active != index:
                return
            if level == 1 and self.echo_rise is None:
                self.echo_rise = tick
            elif level == 0 and self.echo_rise is not None:
                self.echo_fall = tick
                self.echo_done.set()

    def ultrasound(self):
        cfg = self.cfg['ultrasonic']
        try:
            while not self.shutdown.is_set():
                for i, s in enumerate(cfg['sensors']):
                    start = time.monotonic()
                    with self.lock:
                        self.echo_active, self.echo_rise, self.echo_fall = i, None, None
                        self.echo_done.clear()
                    stuck = self.lg.gpio_read(self.handle, s['echo']) != 0
                    if not stuck:
                        # Kernel-scheduled finite wave, not Python microsecond sleep/busy-loop.
                        self.lg.tx_pulse(self.handle, s['trig'], 10, 10, 0, 1)
                        self.echo_done.wait(cfg['timeout_s'])
                    with self.lock:
                        self.echo_active = None
                        distance = None
                        status = 'echo_stuck_high' if stuck else 'timeout'
                        if self.echo_fall is not None:
                            candidate = (self.echo_fall - self.echo_rise) * 1e-9 * cfg['speed_of_sound_m_s'] / 2
                            if cfg['min_m'] <= candidate <= cfg['max_m']:
                                distance, status = candidate, 'ok'
                            else:
                                status = 'out_of_range'
                        self.ranges[i] = {'name': s['name'], 'm': distance, 'status': status, 'at': time.monotonic()}
                    if self.shutdown.wait(max(0, cfg['interval_s'] - (time.monotonic() - start))):
                        return
        except Exception as exc:
            self.sensor_error = str(exc)

    def motor(self, index, pwm):
        if not self.cfg['hardware']['enabled']:
            if pwm:
                raise RuntimeError('motor output disabled')
            return
        m, h = self.cfg['hardware']['motors'][index], self.cfg['hardware']
        pwm *= m['direction']
        now = time.monotonic()
        # Renew an expiring train. Cancel first, so the lgpio PWM FIFO cannot extend a stale lease.
        if pwm == self.previous_pwm[index] and pwm and now < self.burst_until[index] - 0.025:
            return
        for pin in m['pwm']:
            self.lg.tx_pwm(self.handle, pin, 0, 0)
            self.lg.gpio_write(self.handle, pin, 0)
        if pwm:
            pin = m['pwm'][0 if pwm > 0 else 1]
            cycles = max(1, int(h['pwm_hz'] * h['pwm_lease_s']))
            # Never allow 100%-duty static HIGH: a finite train must end LOW.
            self.lg.tx_pwm(self.handle, pin, h['pwm_hz'], min(abs(pwm) * 100, 99), 0, cycles)
            self.burst_until[index] = now + cycles / h['pwm_hz']
        self.previous_pwm[index] = pwm

    def stop(self):
        errors = []
        for i in range(2):
            try:
                self.motor(i, 0)
            except Exception as exc:
                errors.append(str(exc))
        if self.servo and not self.cfg['servo']['hold_on_stop']:
            self.servo.disable()
        if errors:
            raise RuntimeError('; '.join(errors))

    def sample(self, now):
        dt = now - self.last_measure
        motors = []
        with self.lock:
            for i, (m, q) in enumerate(zip(self.cfg['hardware']['motors'], self.encoders)):
                d = {'count': None, 'rpm': None, 'rotor_rpm': None, 'revolutions': None,
                     'wheel_revolutions': None, 'distance_m': None, 'path_m': None,
                     'speed_m_s': None, 'direction': None, 'invalid': None, 'edge_age_s': None,
                     'calibrated': encoder_calibrated(m), 'observed': False}
                if q:
                    delta = q.count - self.last_counts[i]
                    self.last_counts[i] = q.count
                    d.update(count=q.count, invalid=q.invalid, observed=q.last_edge is not None,
                             edge_age_s=None if q.last_edge is None else now - q.last_edge)
                    if encoder_calibrated(m) and q.last_edge is not None:
                        rev = q.count / m['counts_per_motor_rev']
                        wheel_delta = delta / m['counts_per_motor_rev'] / m['gear_ratio']
                        self.total_path[i] += abs(wheel_delta) * math.pi * self.cfg['hardware']['wheel_diameter_m']
                        rpm = wheel_delta * 60 / dt
                        d.update(rpm=rpm, rotor_rpm=rpm * m['gear_ratio'], revolutions=rev,
                                 wheel_revolutions=rev / m['gear_ratio'],
                                 distance_m=rev / m['gear_ratio'] * math.pi * self.cfg['hardware']['wheel_diameter_m'],
                                 path_m=self.total_path[i], speed_m_s=rpm / 60 * math.pi * self.cfg['hardware']['wheel_diameter_m'],
                                 direction=1 if delta > 0 else -1 if delta < 0 else 0)
                motors.append(d)
            trailer = {'angle_rad': None, 'count': None, 'zeroed': False, 'index_count': self.index_count, 'invalid': None}
            if self.trailer:
                t, q = self.cfg['trailer'], self.trailer
                if self.index_pending is not None:
                    if t['index_angle_rad'] is not None:
                        self.zero_count, self.zero_angle = self.index_pending, t['index_angle_rad']
                    self.index_pending = None
                trailer.update(count=q.count, invalid=q.invalid, zeroed=self.zero_count is not None,
                               sampled_at=now, last_edge_at=q.last_edge)
                if self.zero_count is not None and not q.invalid:
                    angle = (q.count - self.zero_count) * 2 * math.pi / (4 * t['ppr']) + self.zero_angle + t['offset_rad']
                    trailer['angle_rad'] = math.atan2(math.sin(angle), math.cos(angle))
            ranges = [dict(r, age_s=None if r['at'] is None else now-r['at']) for r in self.ranges]
            for r in ranges:
                if r['age_s'] is not None and r['age_s'] > 1.0:
                    r.update(m=None, status='stale')
        self.last_measure = now
        return motors, trailer, ranges

    def close(self):
        self.shutdown.set()
        if self.sensor_thread:
            self.sensor_thread.join(timeout=0.3)
        if self.handle is not None:
            try:
                self.stop()
            except Exception:
                pass
            for cb in self.callbacks:
                cb.cancel()
            self.lg.gpiochip_close(self.handle)
            self.handle = None
        if self.servo:
            self.servo.disable()
