"""Read-only checks: does not claim pins or send motor/servo pulses."""
import importlib.util
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/rc_car'))
from rc_car.core import load_config, odom_configured


def main():
    cfg = load_config(sys.argv[1] if len(sys.argv) > 1 else 'config/vehicle.yaml')
    print('Configuration syntax/constraints: OK')
    for name in ('rclpy', 'lgpio', 'aiohttp', 'yaml'):
        print(f'{name}: {"available" if importlib.util.find_spec(name) else "NOT installed in this environment"}')
    print('Motor outputs:', 'ENABLED BY CONFIGURATION' if cfg['hardware']['enabled'] else 'DISABLED')
    print('Encoder odometry configuration:', 'complete; hardware NOT verified here' if odom_configured(cfg) else 'incomplete; no odom/TF published')
    paths = [f"/dev/gpiochip{cfg['hardware']['gpiochip']}", cfg['servo']['pwmchip'], cfg['lidar']['serial_port'], cfg['slam']['params_file']]
    for path in paths:
        print(f'{path}: exists={Path(path).exists()}, read={os.access(path, os.R_OK)}, write={os.access(path, os.W_OK)}')
    print('AUTO_PARK: locked in code. No algorithm is installed.')
    print('This check does NOT verify pinmux, encoder electrical levels, timing, motor direction, or SLAM.')


if __name__ == '__main__':
    main()
