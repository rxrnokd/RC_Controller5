#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
if [[ -f /project/ros2_ws/install/setup.bash ]]; then
  source /project/ros2_ws/install/setup.bash
fi
if [[ -n "${RC_UNDERLAY_SETUP:-}" ]]; then
  source "$RC_UNDERLAY_SETUP"
fi
cd /project/ros2_ws
packages=(rc_car)
if ! ros2 pkg prefix sllidar_ros2 >/dev/null 2>&1; then
  test -f src/sllidar_ros2/package.xml || { echo 'Existing sllidar_ros2 source is missing.' >&2; exit 1; }
  packages=(sllidar_ros2 rc_car)
fi
ros2 pkg prefix slam_toolbox >/dev/null
# Isolated output leaves the existing workspace's build/install/log intact.
colcon --log-base /project/.rc_log build --base-paths src \
  --build-base /project/.rc_build --install-base /project/.rc_install \
  --packages-select "${packages[@]}" --symlink-install
source /project/.rc_install/setup.bash
python3 /project/deploy/preflight.py /project/config/vehicle.yaml
exec ros2 launch rc_car vehicle.launch.py config:=/project/config/vehicle.yaml
