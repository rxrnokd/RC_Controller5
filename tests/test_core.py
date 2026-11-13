"""Safety/units regression tests; synthetic inputs only, never a sensor simulator."""
import copy
import math
from pathlib import Path
import sys
import tempfile
import unittest

import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/rc_car'))
from rc_car.core import (CommandManager, PI, Quadrature, dumps, encoder_calibrated,
                         integrate, load_config, number, odom_configured, servo_pulse, wheel_targets)


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(Path(__file__).resolve().parents[1] / 'config/vehicle.yaml')
        self.cfg['hardware']['max_pwm'] = 0.4
        for m in self.cfg['hardware']['motors']:
            m['max_rpm'] = 100
        self.cfg['servo']['max_angle_rad'] = 0.4
        self.manager = CommandManager(self.cfg)
        self.feedback = {'ready': True, 'pi_ready': True, 'motors': [{'pwm': 0}, {'pwm': 0}], 'fault': None, 'epoch': 0}

    def send(self, kind, now=10, session='a', **kwargs):
        self.manager.handle(dict(type=kind, session=session, received=now, seq=1, **kwargs), now, self.feedback)

    def arm(self):
        self.send('claim')
        self.send('arm')

    def test_default_no_outputs_no_odometry(self):
        cfg = load_config('config/vehicle.yaml')
        self.assertFalse(cfg['hardware']['enabled'])
        self.assertEqual(cfg['hardware']['max_pwm'], 0)
        self.assertFalse(odom_configured(cfg))

    def test_default_estop_requires_arm(self):
        self.send('claim')
        with self.assertRaises(ValueError):
            self.send('drive', pwm=[1, 1], steering_rad=0)
        self.assertTrue(self.manager.estop)

    def test_owner_exclusive(self):
        self.send('claim')
        with self.assertRaises(ValueError):
            self.send('claim', session='b')
        with self.assertRaises(ValueError):
            self.send('arm', session='b')

    def test_any_client_can_estop(self):
        self.arm()
        self.send('estop', session='observer')
        self.assertFalse(self.manager.armed)
        self.assertTrue(self.manager.estop)

    def test_disconnect_releases_and_stops(self):
        self.arm()
        self.send('release')
        self.assertIsNone(self.manager.owner)
        self.assertFalse(self.manager.armed)

    def test_timeout_does_not_auto_rearm(self):
        self.arm()
        self.send('drive', pwm=[0.1, 0.1], steering_rad=0)
        result = self.manager.tick(10.31, self.feedback)
        self.assertIsNone(result['target'])
        with self.assertRaises(ValueError):
            self.send('drive', now=10.32, pwm=[0.1, 0.1], steering_rad=0)

    def test_missing_feedback_stops(self):
        self.arm()
        self.assertFalse(self.manager.tick(10.01, None)['armed'])

    def test_auto_rejected_and_stops(self):
        self.arm()
        with self.assertRaises(ValueError):
            self.send('mode', mode='AUTO_PARK')
        self.assertFalse(self.manager.armed)
        self.assertEqual(self.manager.mode, 'MANUAL')

    def test_control_mode_change_stops(self):
        self.arm()
        self.send('control', control='PI')
        self.assertFalse(self.manager.armed)

    def test_pi_not_armed_without_feedback(self):
        self.send('claim')
        self.send('control', control='PI')
        self.feedback['pi_ready'] = False
        with self.assertRaises(ValueError):
            self.send('arm')

    def test_fault_never_falls_back(self):
        self.arm()
        self.manager.control = 'PI'
        self.feedback.update(fault='encoder_stall', epoch=self.manager.epoch)
        result = self.manager.tick(10.01, self.feedback)
        self.assertFalse(result['armed'])
        self.assertEqual(result['control'], 'PI')

    def test_old_controller_fault_requires_new_epoch(self):
        self.arm()
        self.feedback.update(fault='timeout', epoch=0)
        self.assertTrue(self.manager.tick(10.01, self.feedback)['armed'])

    def test_limits_enforced(self):
        self.arm()
        self.send('drive', pwm=[99, -99], steering_rad=99)
        self.assertEqual(self.manager.target, {'values': [0.4, -0.4], 'steering_rad': 0.4})

    def test_limit_escalation_rejected(self):
        self.arm()
        with self.assertRaises(ValueError):
            self.send('limits', pwm=0.9, rpm=[10, 10], steering=0.1)
        self.assertFalse(self.manager.armed)

    def test_nan_inf_bool_rejected(self):
        for bad in (float('nan'), float('inf'), -float('inf'), True, '10', None):
            with self.assertRaises(ValueError):
                number(bad)
        with self.assertRaises(ValueError):
            dumps({'rpm': float('nan')})

    def test_old_request_rejected(self):
        with self.assertRaises(ValueError):
            self.manager.handle({'type': 'claim', 'session': 'a', 'received': 1}, 2, self.feedback)

    def test_wheel_targets_units_and_ratio(self):
        self.cfg['geometry'].update(verified=True, wheelbase_m=0.3, track_m=0.2)
        straight = wheel_targets(math.pi*0.075, 0, self.cfg)
        self.assertEqual(straight, [60, 60])
        left, right = wheel_targets(1, 0.3, self.cfg)
        self.assertGreater(right, left)

    def test_extreme_speed_overflow_rejected(self):
        self.cfg['geometry'].update(verified=True, wheelbase_m=0.3, track_m=0.2)
        with self.assertRaises(ValueError):
            wheel_targets(1e308, 0, self.cfg)

    def test_odom_straight_reverse_and_arc(self):
        x, y, a = integrate(0, 0, 0, 1, 0, 0.3)
        self.assertEqual((x, y, a), (1, 0, 0))
        self.assertEqual(integrate(x, y, a, -1, 0, 0.3), (0, 0, 0))
        x, y, a = integrate(0, 0, 0, math.pi/2, math.atan(0.3), 0.3)
        self.assertAlmostEqual(x, 1)
        self.assertAlmostEqual(y, 1)
        self.assertAlmostEqual(a, math.pi/2)

    def test_quadrature_x4_forward_reverse(self):
        q = Quadrature()
        for channel, level in ((1, 1), (0, 1), (1, 0), (0, 0)):
            q.edge(channel, level, 1)
        self.assertEqual(q.count, 4)
        for channel, level in ((0, 1), (1, 1), (0, 0), (1, 0)):
            q.edge(channel, level, 2)
        self.assertEqual(q.count, 0)

    def test_motor_trailer_independent_counts(self):
        motor, trailer = Quadrature(), Quadrature()
        trailer.edge(1, 1, 1)
        self.assertEqual(motor.count, 0)
        self.assertEqual(trailer.count, 1)

    def test_pi_saturation_antiwindup_slew(self):
        pi = PI(0.1, 1)
        for i in range(100):
            output = pi.step(100, 0, 0.02, 0.3, 1, i*0.02, 0.2)
            self.assertLessEqual(output, 0.3)
        self.assertEqual(pi.integral, 0)
        self.assertEqual(pi.step(0, 0, 0.02, 0.3, 1, 3, 0.2), 0)

    def test_pi_uses_measured_feedback(self):
        a, b = PI(0.01, 0), PI(0.01, 0)
        slow = a.step(20, 0, 1, 0.5, 1, 1, 0.2)
        fast = b.step(20, 19, 1, 0.5, 1, 1, 0.2)
        self.assertGreater(slow, fast)

    def test_reverse_has_zero_dwell(self):
        pi = PI(0, 0)
        self.assertGreater(pi.step(0.3, 0, 1, 0.4, 1, 1, 0.2, True), 0)
        self.assertEqual(pi.step(-0.3, 0, 0.02, 0.4, 1, 1.02, 0.2, True), 0)
        self.assertEqual(pi.step(-0.3, 0, 0.02, 0.4, 1, 1.1, 0.2, True), 0)
        self.assertLess(pi.step(-0.3, 0, 0.02, 0.4, 1, 1.3, 0.2, True), 0)

    def test_lower_limit_immediately_clamps(self):
        pi = PI(0, 0)
        pi.step(0.4, 0, 1, 0.4, 1, 1, 0.2, True)
        self.assertLessEqual(pi.step(0.4, 0, 0.02, 0.1, 1, 2, 0.2, True), 0.1)

    def test_servo_table_maps_road_angle(self):
        table = [[-0.3, 1200], [0, 1510], [0.3, 1780]]
        self.assertEqual(servo_pulse(table, 0), 1510)
        self.assertEqual(servo_pulse(table, 0.15), 1645)
        with self.assertRaises(ValueError):
            servo_pulse(table, 0.4)

    def test_pin_conflict_rejected(self):
        cfg = copy.deepcopy(self.cfg)
        cfg['trailer']['a'] = 13
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'bad.yaml'
            path.write_text(yaml.safe_dump(cfg), encoding='utf-8')
            with self.assertRaises((ValueError, AssertionError)):
                load_config(path)


if __name__ == '__main__':
    unittest.main()
