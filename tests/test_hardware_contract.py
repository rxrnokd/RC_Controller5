"""GPIO API contract tests using a call recorder; no physical/simulated vehicle."""
from pathlib import Path
import sys
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/rc_car'))
from rc_car.core import load_config, Quadrature
from rc_car.hardware import Hardware


class Recorder:
    def __init__(self):
        self.calls = []

    def tx_pwm(self, *args):
        self.calls.append(('pwm', args))

    def gpio_write(self, *args):
        self.calls.append(('write', args))


class HardwareContract(unittest.TestCase):
    def setUp(self):
        self.hw = Hardware.__new__(Hardware)
        self.hw.cfg = load_config('config/vehicle.yaml')
        self.hw.cfg['hardware']['enabled'] = True
        self.hw.handle, self.hw.lg = 0, Recorder()
        self.hw.previous_pwm, self.hw.burst_until = [0, 0], [0, 0]
        self.hw.servo = None

    def test_each_active_train_is_finite_and_ends_low(self):
        self.hw.motor(0, 0.4)
        kind, args = self.hw.lg.calls[-1]
        self.assertEqual(kind, 'pwm')
        self.assertGreater(args[-1], 0)
        self.assertLessEqual(args[3], 99)
        self.assertLessEqual(args[-1]/args[2], 0.15)

    def test_stop_sets_both_inputs_low_for_both_motors(self):
        self.hw.motor(0, 0.2)
        self.hw.motor(1, -0.2)
        self.hw.lg.calls.clear()
        self.hw.stop()
        zeros = [args[1] for kind, args in self.hw.lg.calls if kind == 'write' and args[2] == 0]
        self.assertEqual(zeros, [16, 20, 21, 26])

    def test_changing_output_cancels_queued_train_first(self):
        self.hw.motor(0, 0.2)
        self.hw.lg.calls.clear()
        self.hw.motor(0, 0.3)
        self.assertEqual(self.hw.lg.calls[0], ('pwm', (0, 16, 0, 0)))
        self.assertEqual(self.hw.lg.calls[2], ('pwm', (0, 20, 0, 0)))

    def test_gpio_output_disabled_is_fail_closed(self):
        self.hw.cfg['hardware']['enabled'] = False
        with self.assertRaises(RuntimeError):
            self.hw.motor(0, 0.1)
        self.assertEqual(self.hw.lg.calls, [])

    def test_repeated_edge_marks_capture_loss(self):
        q = Quadrature()
        q.edge(1, 1, 1)
        q.edge(1, 1, 2)
        self.assertEqual(q.invalid, 1)

    def test_trailer_zero_does_not_reset_motor_counter(self):
        self.hw.lock = threading.RLock()
        self.hw.trailer = Quadrature()
        self.hw.trailer.count = 123
        self.hw.encoders = [Quadrature(), Quadrature()]
        self.hw.encoders[0].count = 500
        self.hw.zero_trailer()
        self.assertEqual(self.hw.zero_count, 123)
        self.assertEqual(self.hw.encoders[0].count, 500)


if __name__ == '__main__':
    unittest.main()
