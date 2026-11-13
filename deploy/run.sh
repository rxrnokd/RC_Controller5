#!/usr/bin/env bash
# Run on Raspberry Pi OS from ~/rc_lidar_docker; no Docker/ROS installation.
set -euo pipefail
cd "$(dirname "$0")/.."
image="${RC_IMAGE:?Set RC_IMAGE to your existing image extended with Dockerfile.control}"
args=(--name rc-car --restart unless-stopped --network host --init
      --cap-drop ALL --security-opt no-new-privileges
      -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e ROS_DOMAIN_ID=42
      --mount "type=bind,src=$(pwd),dst=/project")
if [[ "${1:-}" == '--hardware' ]]; then
  gpio="${GPIO_DEVICE:-/dev/gpiochip0}"
  test -c "$gpio" || { echo "Missing $gpio" >&2; exit 1; }
  args+=(--device "$gpio:$gpio")
  # Only the selected PWM chip is writable; no /dev/mem or --privileged.
  pwm="${PWM_CHIP_PATH:-/sys/class/pwm/pwmchip0}"
  test -d "$pwm/pwm1" || { echo 'Export PWM channel 1 on the host first; see README.' >&2; exit 1; }
  args+=(--mount "type=bind,src=$(readlink -f "$pwm"),dst=/sys/class/pwm/pwmchip0")
fi
if [[ -n "${LIDAR_DEVICE:-}" ]]; then
  test -c "$LIDAR_DEVICE" || { echo "Missing $LIDAR_DEVICE" >&2; exit 1; }
  args+=(--device "$LIDAR_DEVICE:/dev/ttyUSB0")
fi
# Existing containers are deliberately not removed or overwritten.
docker run -d "${args[@]}" "$image"
