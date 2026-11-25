# Raspberry Pi 4 RC 차량 · ROS 2 Jazzy

기본 구성은 DC 모터 엔코더를 사용하지 않는다. Raspberry Pi가 RPLIDAR C1 스캔으로 차량 위치와 방향을 추정하고 웹 수동 조종에는 Open-loop PWM을 사용한다. 트레일러 연결각은 MT6701로 측정한다. 노트북에서는 주차 경로를 계획할 수 있으며 실제 자동주차 구동은 잠겨 있다.

기본 설정은 모터/서보/센서/라이다/SLAM 비활성, 최대 PWM 0, 비상정지 래치 상태다. 설정을 채우기 전에도 기존 ROS 환경에서 웹 서버와 상태 노드를 실행할 수 있다. 없는 측정값은 JSON `null`, UI `—`로 표시한다. 임의의 지도·RPM·거리·차량 위치를 만들지 않는다.

## 1. 제공 자료 분석과 확인 범위

현재 작업 폴더에 실제 `Dockerfile`, `config/slam.yaml`, `sllidar_ros2` 소스, `maps` 내용은 **없었다**. 폴더 목록 TXT로 존재 구조만 확인했다. 따라서 기존 이미지 태그·컨테이너 이름·원래 SLAM 파라미터·설치 경로는 Pi에서 아래 절차로 확인해야 한다. 이 구현은 해당 파일을 덮어쓰거나 ROS/SLAM/드라이버를 새로 내려받지 않는다.

| 제공 자료 | 확인한 내용 | 구현에 반영 / 미확정 사항 |
|---|---|---|
| `GPIO_Pin.txt` | BCM 핀 배치, US-100 모델, MT6701 ABZ | 아래 배선표로 반영. 실제 커넥터 방향·전압·풀업·pinmux는 미검증 |
| `도커 ROS SLAM 폴더 구조.txt` | Dockerfile, config/slam.yaml, maps, ros2_ws/src/sllidar_ros2, C1 launch 및 SDK 포함 | 기존 소스와 설치 환경 재사용. 파일 본문은 제공되지 않음 |
| `스크린샷 2026-11-08 081911.png` | JGB37-520R30-12, 12V, 감속 1:30, 정격 280RPM, 무부하 320RPM, 정격전류 0.36A, 정지전류 3.2A, AB 엔코더, 11 line, 엔코더 전원 3.3~5V | 280/320은 감속 출력축 자료로 취급. 허용 운전 최대 RPM/PWM은 따로 검증하여 입력. 6핀 순서와 출력회로는 그림만으로 확정 불가 |
| `스크린샷 2026-11-08 081923.png` | LD-1501MG, 6~8.4V, 500~2500μs, 0~180°, 정지전류 2.4~3A | 이 범위는 서보 자체 사양이며 타이어 조향각이 아님. 링크 간섭 없는 펄스·실제 조향각 보정표 필요 |
| `MDD3A_Datasheet.pdf` | 인쇄 p.2: 4~16V, 채널당 연속 3A / 5초 미만 피크 5A, 입력 HIGH 1.7~12V, PWM 최대 20kHz, 5V 보조출력 200mA. p.3 진리표를 렌더링하여 확인 | A/B = LOW/LOW는 브레이크, HIGH/LOW 정방향, LOW/HIGH 역방향. 정지 시 LOW/LOW. 전류 제한 센서는 없음 |
| `SLAMTEC_rplidar_datasheet_C1_v1.2_en.pdf` | C1M1, UART 460800 8N1, 전원 4.8~5.2V, 시작전류 전형 800mA, 동작 230~260mA, 5k sample/s, 전형 10Hz, 흰색 물체 0.05~12m | 기존 공식 C1 launch 재사용. 드라이버의 좌표 변환을 다시 뒤집지 않음. 장착 TF는 실측값만 사용 |
| `SLAMTEC_rplidarkit_usermanual_C1_v1.2_en.pdf` | USB 어댑터 CP2102, USB-C 연결, 스캔 시작/정지와 상태 확인, 2분 이상 예열 권고 | USB-UART 경로 사용. 기존 컨테이너와 새 노드가 동시에 같은 장치를 열지 않도록 운영 |
| `LR002_SLAMTEC_rplidar_sdk_v2.0_en.pdf` | SDK 구조, Linux/Windows 빌드, 채널 연결, startScan/grabScanDataHq/getHealth | 기존 sllidar_ros2 내 SDK를 사용. 오래된 일반 예제의 A2/A3 baudrate를 C1에 적용하지 않음 |

동일 모델에 대한 [HiWonder 공식 섀시 문서](https://docs.hiwonder.com/projects/Ackermann-Chassis/en/latest/docs/2_STM32_Version_checked.html)는 JGB37-520R30-12 엔코더를 `11 × 4 × 30 = 1320 count/감속 출력축 회전`, LD-1501MG를 20ms PWM 주기로 설명한다. 이를 근거로 `counts_per_motor_rev=44`, 감속비 30, 서보 50Hz를 참고 설정으로 넣었다. **사용자가 가진 Byte Robot 표기 제품과 같은 전기적 변형인지 확인하지 못했으므로 검증 플래그는 false**다. 실제 출력축 1회전 카운트와 배선 전압을 확인한 뒤 활성화한다. 판매 이미지의 모터 허용 전류와 드라이버 연속 전류는 서로 다르므로 장시간 스톨 운전을 허용하면 안 된다.

[MT6701 제조사 데이터시트](https://www.novosns.com/enfiles/MT6701_Rev.1.8.pdf)의 ABZ 해상도는 EEPROM으로 설정하는 1~1024 PPR다. 실제 설정을 모르므로 `trailer.ppr=null`로 유지했다. **MT6701은 트레일러용이며 모터 RPM 계산에 사용하지 않는다.** ABZ만으로 부팅 당시 절대각을 알 수 없다.

[US-100 공급사 문서](https://www.adafruit.com/product/4019)에 따르면 점퍼를 제거하면 Trigger/Echo 방식, 점퍼 장착 시 UART 방식이다. 현재 배선은 Trigger/Echo다. 3.3V 전원 및 신호를 우선 검증하고, 5V Echo는 레벨 변환 없이 Pi GPIO에 연결하지 않는다. [Raspberry Pi GPIO 설명](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)과 실제 센서 출력 전압을 함께 확인한다.

## 2. 구조와 파일

```text
노트북 브라우저 + 별도로 실행하는 노트북 Python 주차 알고리즘
        │ 각 클라이언트에서 같은 Pi 포트로 WebSocket 한 개씩 /ws
        ▼
Docker web_bridge ── /rc/request ──► command_manager
        ▲                             │ /rc/command
        │                             ▼
        ├── /rc/state ◄────── vehicle_controller ── GPIO / 커널 PWM
        │                         │
        │                         ├─ DC 모터 2개 · 수동 PWM
        │                         ├─ MT6701 ABZ (트레일러 전용)
        │                         └─ US-100 4개
        │
        ├── /odom ◄──── icp_odometry ◄── /scan
        └── /map ◄──── slam_toolbox ◄── /scan (기존 sllidar_ros2)

TF: map ──SLAM Toolbox──► odom ──라이다 스캔 정합──► base_link ──실측 고정 TF──► laser
```

모터 제어, 명령 관리, 웹 서버, 라이다 Odometry는 별도 프로세스에서 실행한다. 모터 루프는 독립적으로 명령 만료를 확인한다. Docker 내부 통신은 ROS 토픽을 사용하고 노트북은 WebSocket으로 연결한다.

| 파일 | 역할 |
|---|---|
| `config/vehicle.yaml` | 차량 하드웨어·보정·제한·네트워크 설정의 단일 원본 |
| `ros2_ws/src/rc_car/rc_car/core.py` | 설정 검증, 제어권/안전 상태, PI, 카운팅, 운동학 |
| `.../hardware.py` | Pi lgpio, 커널 PWM, ABZ, 초음파 실측 |
| `.../nodes.py` | Command Manager, 차량 제어, Odometry 실행 파일 |
| `.../web.py`, `.../web/*` | ROS↔단일 WebSocket, 지도 저장, 웹 UI |
| `.../launch/vehicle.launch.py` | 기존 C1/SLAM launch 통합 |
| `deploy/*` | 기존 이미지 확장, 점검, 실행 |
| `laptop/client.py` | 웹과 같은 프로토콜을 쓰는 비동기 클라이언트. 직접 실행하면 관찰만 수행 |
| `laptop/parking/` | 노트북 전용. planner.py는 운동학·충돌·Hybrid A*, mpc.py는 계획·추종 MPC, __main__.py는 저장 지도 및 실시간 실행 |
| `.../parking_control.py` | Pi Command Manager의 주차 제어권·피드백·정지 검사 |
| `tests/*` | 하드웨어 독립 회귀 테스트. 센서 시뮬레이터가 아님 |

## 3. GPIO, 전원, Docker 접근

모든 숫자는 BCM GPIO 번호다. 물리 헤더 핀 번호와 다르다.

| 장치 | 핀 |
|---|---|
| MDD3A 모터 1 A/B 입력 | 16 / 20 |
| 모터 1 자체 엔코더 A/B | 4 / 14 |
| MDD3A 모터 2 A/B 입력 | 21 / 26 |
| 모터 2 자체 엔코더 A/B | 15 / 18 |
| 조향 서보 PWM | 13 |
| MT6701 A/B/Z | 19 / 10 / 9 |
| US-100 1 Trigger/Echo | 17 / 27 |
| US-100 2 Trigger/Echo | 22 / 23 |
| US-100 3 Trigger/Echo | 24 / 25 |
| US-100 4 Trigger/Echo | 5 / 6 |

GPIO14/15는 UART, GPIO9/10은 SPI0, GPIO4는 1-wire, GPIO18/19/20/21은 PCM/I2S 등과 충돌할 수 있다. 해당 기능의 실제 활성 상태를 확인한다. GPIO19를 PWM으로 설정하면 MT6701 A와 충돌한다. 서보에는 GPIO13 하나만 PWM으로 지정한다.

Pi, 센서, 모터 드라이버의 신호 GND는 공통으로 연결한다. 모터 12V 전원, 서보용 적정 전압의 충분한 전류 전원, Pi 5V 전원을 구분한다. MDD3A의 5VO 200mA로 Pi·서보·라이다를 공급하지 않는다. 모터 전원 입력에 적정 퓨즈와 즉시 전원을 끊는 물리적 정지 장치를 설치하고, MDD3A 입력은 프로세스가 없거나 Pi 부팅 중에도 LOW가 되도록 외부 풀다운을 검토한다. 소프트웨어는 전류를 측정하지 않으며 PWM 제한이 과전류 보호를 대신하지 않는다.

### Raspberry Pi OS에서 1회 확인

```bash
cd ~/rc_lidar_docker
uname -m                       # aarch64 확인
docker ps -a
docker images
cat Dockerfile
cat config/slam.yaml
ls ros2_ws/src/sllidar_ros2/launch/sllidar_c1_launch.py
ls -l /dev/gpiochip* /dev/serial/by-id/
gpioinfo                       # 설치되어 있다면 실제 gpiochip/line 소유자 확인
pinctrl get 4-27                # OS에 해당 도구가 있는 경우
cat /boot/firmware/config.txt
cat /boot/firmware/cmdline.txt
```

모터 전원을 분리한 상태에서 필요 없는 UART 콘솔/하드웨어 UART, SPI, 1-wire, I2S/PWM 오디오 점유를 해제한다. 설정을 통째로 덮어쓰지 말고 기존 기능을 확인하여 편집한다. Pi OS 버전에 따라 부팅 설정 경로가 `/boot/config.txt`일 수 있다. `raspi-config`의 Serial Port 메뉴에서 로그인 셸과 UART를 비활성화하고 재부팅하여 GPIO14/15를 점검한다.

GPIO13 커널 PWM용 설정 예시는 다음과 같다. 기존 오디오/PWM 설정과 충돌하지 않는지 확인한다. 파라미터 의미는 [Raspberry Pi 공식 overlay 목록](https://github.com/raspberrypi/linux/blob/rpi-6.12.y/arch/arm/boot/dts/overlays/README)의 `pwm` 항목을 참고한다.

```ini
dtparam=audio=off
dtoverlay=pwm,pin=13,func=4
```

재부팅 후 **실제 pwmchip 매핑**을 확인한다. `pwmchip0/channel1`은 이 코드의 기본값일 뿐 장치 식별 결과를 대신하지 않는다. Pi 4 GPIO13의 하드웨어 PWM 채널은 1이며 gpiochip과 pwmchip은 서로 다른 번호 체계다.

```bash
ls -l /sys/class/pwm
cat /sys/class/pwm/pwmchip0/npwm
readlink -f /sys/class/pwm/pwmchip0
# 올바른 chip임을 확인한 뒤 export만 수행 (enable/pulse 출력은 하지 않음)
test -d /sys/class/pwm/pwmchip0/pwm1 || echo 1 | sudo tee /sys/class/pwm/pwmchip0/export
```

Docker는 호스트 커널과 GPIO 장치를 공유한다. GPIO edge/PWM에는 선택한 `/dev/gpiochipN`만 `--device`로 전달하고, 서보에는 선택한 pwmchip 실제 디렉터리만 쓰기 가능한 bind mount로 제공한다. USB 라이다는 해당 USB serial 장치만 제공한다. `/dev/mem`, 전체 `/dev`, `--privileged`를 사용하지 않는다. 컨테이너는 root이지만 capability는 모두 제거한다. 호스트 장치 소유권과 cgroup 제한이 여전히 적용되며 **이 권한 조합의 실제 Pi 실행은 미검증**이다.

모터는 [lgpio API](https://github.com/joan2937/lg/blob/master/PY_LGPIO/lgpio_extra.py)의 software-timed PWM과 커널 edge 알림을 사용한다. 1kHz는 시운전용 소프트웨어 설정이며 모터 최적 주파수라고 보증하지 않는다. 라이브러리 상한 10kHz가 MDD3A의 20kHz보다 낮다. 서보는 같은 라이브러리의 software-timed servo를 쓰지 않고 커널 hardware PWM을 사용한다. SLAM·지도 전송 부하에서 파형과 카운트 손실을 반드시 측정한다.

## 4. 코드 전송 및 기존 이미지 재사용

### Windows 개발 환경 (PowerShell)

아래 사용자명과 주소를 실제 Pi 값으로 바꾼다. 기존 프로젝트 디렉터리가 이미 있는 것을 전제로 한다.

```powershell
$PiTarget = '사용자명@라즈베리파이IP'
scp -r deploy "${PiTarget}:~/rc_lidar_docker/"
scp -r ros2_ws/src/rc_car "${PiTarget}:~/rc_lidar_docker/ros2_ws/src/"
scp config/vehicle.yaml "${PiTarget}:/tmp/rc-vehicle-template.yaml"
scp README.md "${PiTarget}:~/rc_lidar_docker/README.md"
```

처음 설치하면 아래 명령이 기본 설정을 복사한다. 기존 `config/vehicle.yaml`을 사용 중이면 보정값을 보존하고 `geometry` 아래에 `odom_source: lidar`를 추가한다. 두 모터의 `encoder_enabled`는 `false`로 설정하고 `parking:` 절이 없으면 템플릿에서 복사한다. 출처 설정이 없는 이전 파일은 기존 wheel 방식을 사용한다.

```bash
cd ~/rc_lidar_docker
mkdir -p config
test -e config/vehicle.yaml || cp /tmp/rc-vehicle-template.yaml config/vehicle.yaml
```

Windows에서 가능한 점검:

```powershell
python -m unittest discover -s tests -v
python -m compileall -q ros2_ws/src/rc_car laptop deploy
python deploy/preflight.py config/vehicle.yaml
node --check ros2_ws/src/rc_car/web/app.js
```

기본 테스트에는 PyYAML이 필요하다. 노트북 경로 계산 테스트에는 아래 7절의 수치 라이브러리도 설치한다. Windows/macOS에서 웹 수동 조종만 사용할 때는 브라우저로 접속하면 된다.

### Raspberry Pi OS: 이미지 확인과 추가 의존성

기존 컨테이너에서 실제 환경을 먼저 확인한다.

```bash
docker exec -it 기존컨테이너이름 bash
```

**위 셸은 Docker 내부**다.

```bash
source /opt/ros/jazzy/setup.bash
printenv ROS_DISTRO
ros2 pkg prefix slam_toolbox
# 기존 ROS 작업공간의 setup.bash도 source한 후 확인
ros2 pkg prefix sllidar_ros2
ros2 pkg prefix nav2_map_server
python3 -c 'import rclpy; print("rclpy OK")'
exit
```

호스트에서 기존 Jazzy 이미지를 BASE로 파생 이미지를 빌드한다. `deploy/Dockerfile.control`이 라이다 Odometry용 `ros-jazzy-rtabmap-odom`, 지도 저장 및 제어 노드의 의존성을 설치한다. 기존 SLAM Toolbox와 C1 드라이버를 재사용한다. 이미 컨테이너가 있으면 새 이미지로 컨테이너를 만들어야 빌드 결과가 반영된다.

```bash
cd ~/rc_lidar_docker
docker build -f deploy/Dockerfile.control --build-arg BASE_IMAGE=실제기존이미지태그 -t rc-car:local .
```

기존 라이다/SLAM 컨테이너는 새 통합 컨테이너와 동시 실행하지 않는다. 장치 점유 및 중복 `map→odom` TF 문제가 생긴다. 대상 컨테이너를 확인하고 정지한다.

```bash
docker stop 기존컨테이너이름
```

원본 소스는 그대로 둔다. 시작 스크립트는 원래 `ros2_ws/install/setup.bash`가 있으면 source하고, 추가 코드는 `.rc_build`, `.rc_install`, `.rc_log`에 빌드한다. 기존 드라이버가 이미 발견되면 `rc_car`만 빌드한다. 발견되지 않으면 **기존 src/sllidar_ros2**를 함께 빌드하며 clone하지 않는다. 원래 설치가 다른 경로에 있다면 `RC_UNDERLAY_SETUP`을 컨테이너 환경변수로 전달하고 해당 경로를 mount하도록 run.sh를 맞추거나, 제공된 소스를 같은 컨테이너에서 빌드한다. 예전 절대경로를 가리키는 symlink install은 source만으로 복구되지 않을 수 있다.

## 5. 단계별 실행과 보정

### 단계 A: 출력 없이 웹/ROS 확인

`vehicle.yaml`의 모든 `enabled: false`, `max_pwm: 0`을 유지한다. `network.allowed_cidrs`에 **실제 Wi-Fi 서브넷**을 추가한다. 기본값은 localhost만 허용한다. 예를 들어 실제 네트워크가 `192.168.10.0/24`일 때만 그 CIDR을 사용한다. `0.0.0.0/0`은 사용하지 않는다. 라우터 포트포워딩/공용 인터넷 노출은 하지 않는다.

```bash
# Raspberry Pi OS
cd ~/rc_lidar_docker
export RC_IMAGE=rc-car:local
bash deploy/run.sh
docker logs -f rc-car
```

브라우저에서 `http://라즈베리파이IP:8080`에 접속한다. 제어부 상태는 출력 비활성, 지도/센서는 없음이어야 정상이다. 아직 연결되지 않은 하드웨어를 정상으로 표시하지 않는다.

### 단계 B. 선택 사항인 모터 엔코더 점검

라이다 전용 구성에서는 B와 D를 건너뛴다.

모터 전원을 분리하고 바퀴를 들어 올린다. 엔코더 전원, A/B HIGH 전압, GND, PH2.0 6핀 배치를 실측한다. `hardware.wiring_verified=true`로 변경하기 전에 GPIO에 5V가 들어가지 않는지 확인한다. 확인한 모터의 `encoder_enabled=true`만 설정해도 raw count를 읽을 수 있으며 모터 출력 `hardware.enabled`는 false로 유지할 수 있다.

장치를 추가하여 컨테이너를 다시 만들 때는 이전 컨테이너를 정지하고 보관용 이름으로 변경한다. 아래 작업은 Pi에서 사람이 수행한다.

```bash
docker stop rc-car
docker rename rc-car rc-car-monitor-backup
export RC_IMAGE=rc-car:local
export GPIO_DEVICE=/dev/gpiochip0       # gpioinfo로 확인한 실제 장치
export PWM_CHIP_PATH=/sys/class/pwm/pwmchip0
bash deploy/run.sh --hardware
```

이 실행 스크립트는 hardware 프로필에 서보 PWM mount도 준비한다. 서보는 `servo.enabled=false`이면 펄스를 보내지 않는다. 모든 핀이 자유로운 상태인지 먼저 확인한다.

모터 출력축을 정확히 여러 바퀴 회전시켜 카운트 차이를 나눈다. 동일 모델 문서 값이면 출력축 1회전은 1320 count다. `counts_per_motor_rev`는 **감속 전 모터축의 x4 count**, `gear_ratio`는 모터 회전/출력축 회전이다. 출력축 기준 1320을 `counts_per_motor_rev`에 그대로 넣으면 30배 오차가 난다. 모터 내부축/출력축의 기준을 구분하여 검증한다. 전진 시 양의 카운트가 되도록 `encoder_direction`을 결정한다. 두 모터를 각각 검증하고 `encoder_verified`, `gear_ratio_verified`를 true로 바꾼다.

```text
motor_rev = count / counts_per_motor_rev
wheel_rev = motor_rev / gear_ratio
wheel_RPM = Δwheel_rev / Δt × 60
rotor_RPM = wheel_RPM × gear_ratio
signed_distance = wheel_rev × π × 0.075 m
path_length += abs(Δwheel_rev) × π × 0.075 m
```

RPM과 누적 회전·거리는 부팅 이후 값이다. 저장된 절대 주행거리계가 아니다. 처음 유효 edge를 관찰하기 전에는 측정 RPM을 null로 유지한다. 카운트가 일정할 때 0RPM은 카운터의 관측 결과이며 케이블 정상 여부의 증명은 아니다. 정지 중 단선, 완전한 A/B 주기 손실, 정지와 같은 신호 상태는 A/B 신호만으로 완전히 식별할 수 없다. 시작 전 손 회전 점검 및 최대 예상 펄스율에서 기준 계측기와 비교해야 한다. 배선/감속비 보정은 MT6701과 독립이다.

### 단계 C: 서보 및 수동 Open-loop

서보 horn/링크 간섭, 전원과 신호 레벨을 먼저 확인한다. 동일 모델 문서의 50Hz는 참고값이며 실제 제품과 일치하는지 확인한다. `servo.calibration`에는 **타이어 기준의 자전거 모델 조향각(rad)**과 펄스(us) 대응을 최소 3점, 가능하면 여러 점 넣는다. 음수=우회전, 양수=좌회전이다. 예제 숫자를 실제 보정값으로 복사하지 않는다. 다음은 형식만 설명한다.

```text
calibration:
  - [실측_우측_음수각, 실측_펄스]
  - [0.0, 실측_직진_펄스]
  - [실측_좌측_양수각, 실측_펄스]
```

각도는 오름차순, 펄스는 증가 또는 감소 방향으로 단조여야 한다. 실제 제한각보다 보정표 범위가 넓거나 같아야 한다. 서보 자료의 500~2500μs 전체를 조립된 차량에 곧바로 인가하지 않는다. `max_angle_rad`와 검증된 보정표를 채운 후 `servo.calibrated=true`, `servo.enabled=true`로 변경한다. 초기에 실제 기계 위치를 모르므로 첫 펄스의 기계적 이동까지 slew 제한이 보장하는 것은 아니다. 최초에는 바퀴를 들고 안전하게 중심을 맞춘다.

모터·드라이버·전원·물리정지를 확인하고 `hardware.motor_power_verified`, `hardware.external_stop_verified`, `hardware.wiring_verified`를 true로 설정한다. 검증한 낮은 `max_pwm`를 입력한 뒤 `hardware.enabled=true`로 변경한다. 구성상 허용 범위는 0~0.99이며 이는 **안전한 차량 최대 출력이 99%라는 뜻이 아니다**. 무한 HIGH를 피하고 finite PWM train이 LOW로 끝나도록 하는 소프트웨어 상한이다. 실제 허용 최대값은 부하·전원·전류·발열을 측정하여 결정한다. Open-loop에서도 서보가 정상 준비되어야 차량 활성화를 허용한다.

설정 변경 후 `docker restart rc-car`한다. 웹에서 제어권 획득 → Open-loop → 출력 제한/주행 목표 설정 → 정지 해제/활성화 → W/S 또는 ↑/↓. A/D 또는 ←/→로 조향한다. 스페이스/비상정지 버튼은 래치 정지다. 초기 목표 슬라이더는 0%이며 사용자가 올려야 구동한다. 두 모터의 물리적 전진 방향이 다르면 각 `direction`을 맞춘다. 모터 방향과 엔코더 방향은 별도 설정이다.

### 단계 D. 모터 엔코더를 사용하는 차량의 PI 속도 제어

`max_rpm`은 각 모터 **감속 출력축**의 검증된 운전 제한으로 넣는다. 첨부 정격 280RPM 또는 무부하 320RPM을 안전한 최대값으로 자동 적용하지 않았다. `kp`, `ki`는 각각 duty/RPM, duty/(RPM·s)다. 작은 출력·목표에서 먼저 P를 조정한 뒤 I를 조정하며 `pi_verified=true`는 실제 튜닝 후에만 설정한다.

두 엔코더 모두 보정 및 유효 edge 관찰이 되어야 PI를 활성화한다. 정지 → PI 선택 → 활성화 순서를 지킨다. 출력 제한·목표 클램프·적분기 anti-windup·출력 변화율 제한·역전 전 영출력 대기가 적용된다. 조향 변경도 변화율을 제한한다. 목표 0, 비상정지, 통신 만료는 일반 slew를 기다리지 않고 즉시 LOW/LOW를 명령한다.

기본 설정은 50Hz 루프, 0.5초 encoder stall 감시다. 낮은 RPM에서 카운트 간격이 길면 stall 감시의 민감도가 커지므로 검증하여 설정한다. PI 중 예상 움직임에 비해 edge가 사라지거나 반대 방향 피드백/이상 카운트가 확인되면 두 모터를 정지하고 래치한다. **Open-loop로 자동 강등하지 않는다.** 필요하면 사람이 명시적으로 정지 후 Open-loop를 선택하고 재활성화해야 한다.

### 단계 E: MT6701 / 초음파

MT6701의 실제 ABZ 모드·PPR·3.3V 신호·부호를 확인하고 `trailer.ppr`, `verified`, `enabled`를 채운다. 트레일러를 차량과 일직선으로 정렬하고 정지/비활성 상태에서 웹의 영점 버튼을 누른다. 초기 상대각은 영점 이전에 null이다. 재부팅하면 다시 영점 설정해야 한다. `offset_rad`는 영점 후 보정이며 Z 펄스는 기본적으로 횟수만 기록한다. 기계적 Z 기준각을 실측한 경우만 `index_angle_rad`를 설정하면 Z 통과 때 보정한다. 제한된 트레일러 움직임에서는 Z를 지나지 않을 수 있으므로 Z를 보지 못했다는 이유만으로 영점이 있다고 가정하지 않는다. 출력각은 `[-π, π]` rad다.

US-100의 UART 점퍼를 제거하고 전압을 검증한 후 `ultrasonic.electrical_verified`, `enabled`를 true로 설정한다. 네 센서는 65ms 간격으로 순차 측정하여 상호 간섭을 줄인다. 센서별 약 3.8Hz이며 timeout/stuck-high/out-of-range/stale 상태를 구분한다. timeout을 0cm 또는 최대거리로 바꾸지 않는다. WebSocket은 null, ROS Range는 NaN으로 실패를 표현한다. 음속 343m/s는 설정 가능한 근사값이며 온도 측정 기반 보정은 아직 없다. 현재 초음파는 **모니터링용이며 자동 장애물 회피/안전거리 제동은 구현하지 않았다.**

### 단계 F: Odometry / 라이다 / SLAM

기본 `geometry.odom_source: lidar`에서는 C1 `/scan`을 `rtabmap_odom/icp_odometry`가 정합해 `/odom`과 `odom → base_link`를 발행한다. SLAM Toolbox가 같은 스캔과 이 TF로 지도를 만들고 `map → odom`을 제공한다. 기존 wheel 방식은 `geometry.odom_source: wheel`을 선택할 때만 실행한다. [Jazzy ICP 노드](https://github.com/introlab/rtabmap_ros/blob/jazzy-devel/rtabmap_odom/src/nodelets/icp_odometry.cpp)

라이다 위치만 시험할 때는 모터·서보 전원을 끄고 `hardware.enabled`, 두 모터의 `encoder_enabled`, `servo.enabled`, `parking.commissioned`를 `false`로 둔다. `base_link`는 후륜 차축 중심이다. 라이다의 실제 장착 위치·방향을 측정해 아래 TF 설정에 입력한다. 초기에는 `slam.enabled: false`로 Odometry부터 확인한다.

선택 사항인 wheel 방식의 운동학식은 다음과 같다.

```text
v = (v_left + v_right) / 2
yaw_rate = v × tan(steering_angle) / wheelbase
Δyaw = Δdistance × tan(steering_angle) / wheelbase
```

wheel 방식에서는 휠베이스·윤거·엔코더·조향 보정과 `geometry.steering_command_estimate_allowed`가 필요하다. 엔코더 없이 사용하는 lidar 방식에서는 이 값들을 Odometry 입력으로 사용하지 않는다.

`lidar.xyz_rpy`에는 base_link에서 본 라이다 원점의 x/y/z(m), roll/pitch/yaw(rad)를 채우고 `mount_verified=true`로 설정한다. `lidar.enabled=true`로 변경한다. USB 경로는 `/dev/serial/by-id/...`로 식별하고 컨테이너에 `/dev/ttyUSB0`으로 전달한다. `LIDAR_DEVICE`는 run.sh에 주는 호스트 경로이며 `lidar.serial_port`는 컨테이너 경로다. USB 장치 추가 시에는 컨테이너를 정지·이름 변경 후 다시 생성해야 한다.

```bash
# Raspberry Pi OS: 이전 rc-car를 정지하고 보관용 이름으로 바꾼 뒤
export LIDAR_DEVICE=/dev/serial/by-id/실제장치이름
export RC_IMAGE=rc-car:local
bash deploy/run.sh  # 라이다 단독 점검. 모터·서보 GPIO 장치 불필요
docker exec -it rc-car bash
```

**Docker 내부** 점검:

```bash
source /opt/ros/jazzy/setup.bash
source /project/.rc_install/setup.bash
ros2 node list
ros2 pkg executables rtabmap_odom
ros2 topic echo /rc/state --once
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /odom_info_lite --once --field lost --qos-reliability best_effort
ros2 topic echo /rc/odom_status --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
```

모터 전원을 끈 채 바닥에서 차를 천천히 손으로 밀고 돌린다. 줄자로 표시한 직진·후진 거리, 제자리에서 바꾼 방향, 출발점 복귀 오차를 `/odom`과 비교한다. 긴 복도에서 벽이 평행하거나 주변 물체가 움직이면 위치가 흔들리거나 정합이 실패할 수 있다. `/rc/odom_status`의 `valid`가 내려가는지 확인한다. 현장 정확도와 Pi 부하는 아직 측정하지 않았다.

확인 후 `slam.enabled: true`로 변경하고 컨테이너를 재시작한다. `slam.params_file`은 기존 `/project/config/slam.yaml`을 가리킨다. 같은 구역에서 지도와 실측 위치를 비교하고 SLAM 동작 중 `docker stats rc-car`로 Pi 부하도 확인한다. `slam.localization`로 저장 지도를 다시 사용하는 방법은 단계 G를 따른다.

```bash
# Docker 내부
ros2 topic echo /map --once --field info
ros2 run tf2_ros tf2_echo map base_link
ros2 service list | grep slam_toolbox
```

웹의 SLAM 준비 표시는 `/map` 존재, 최근 `/scan`, 최근 Odometry 상태/피드백, 유효하고 최근인 `map→base_link` TF를 모두 요구한다. 이는 입력 준비 여부이며 지도 품질 검증을 의미하지 않는다. 오래된 지도는 마지막 수신 시간과 함께 남겨두되 준비 조건이 깨지면 차량 위치를 숨긴다. `/scan` 전체 거리 배열은 ROS 토픽에서 사용한다. 웹 상태에는 스캔 시각, 프레임, 개수 및 유효 반사점 유무만 보낸다.

### 단계 G: 지도 저장과 재사용

제어권을 가진 상태에서 정지/비활성화하고 지도 이름을 입력한 뒤 저장한다. 이름은 영문/숫자/밑줄/하이픈 1~64자다. 기존 이름은 덮어쓰지 않는다. `/slam_toolbox/save_map`으로 occupancy 이미지/YAML, `/slam_toolbox/serialize_map`으로 `.posegraph`/`.data`를 저장한다. [SLAM Toolbox 지도 저장 구현](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/src/map_saver.cpp)이 사용하는 `nav2_map_server`는 파생 이미지의 추가 의존성이다. 부분 실패/timeout은 오류로 보고하며 서버가 이미 받은 저장 작업은 연결 종료 후에도 끝날 수 있으므로 maps 폴더를 확인한다.

재사용 시 `slam.localization=true`, `slam.posegraph=/project/maps/저장한이름`으로 설정한다. 확장자는 쓰지 않는다. 두 pose graph 파일이 필요하며 PNG/PGM+YAML만으로 SLAM Toolbox localization을 시작하지 않는다. 현재 launch는 `map_start_at_dock=true`이므로 **지도를 처음 만든 시작 자세에 차량을 놓고** 시작한다. 다른 시작 자세 초기화 UI는 아직 제공하지 않는다. 위치가 다른 상태를 자동으로 맞췄다고 간주하면 안 된다.

## 6. 안전 상태와 운전 방식

| 상황 | 동작 |
|---|---|
| 부팅/프로세스 재시작 | MANUAL, 비상정지 래치, 출력 0, 제어권 없음 |
| 제어권 획득/반납 | 정지·비활성화. 획득만으로 움직이지 않음 |
| 새 클라이언트가 이미 점유된 제어권 요청 | 거부. 센서 관찰은 가능 |
| 비상정지 또는 stop | 관찰자를 포함한 모든 연결에서 정지 가능 |
| 모드/PI-Open-loop/제한 변경 | 정지·비활성화 후 별도 활성화 필요 |
| 키 해제 | 주행 목표 0, 모터 즉시 브레이크 |
| 수동 조종 탭 숨김/창 blur/입력칸 포커스 | 활성화된 수동 조종자가 stop 요청. 추가로 차량 측 timeout 적용 |
| WebSocket 종료 | owner release. 전달 실패 시 watchdog |
| 지연된 명령 한 건 | 해당 명령 거부 및 ACK 오류. 세션은 유지하며 마지막 유효 drive의 watchdog을 계속 적용 |
| 마지막 정상 drive 이후 기본 0.30초 | manager 정지·비활성화. heartbeat만으로 연장되지 않음 |
| manager 정지/명령 만료 | controller에서도 독립 확인 후 출력 차단 |
| controller가 멈춤/강제 종료 | 기본 최대 0.10초 finite motor PWM lease. 실제 종료 파형 검증 필요 |
| 센서 준비/출력 초기화 실패 | 활성화 차단, fault 출력 |
| PI에서 엔코더 손실 | 두 모터 정지/래치. 자동 Open-loop 전환 없음 |
| Open-loop에서 엔코더 없음 | 명시적으로 선택한 수동 PWM은 허용. RPM/Odometry 유효성은 별도 처리 |
| AUTO_PARK 요청 | 정지 상태, 실차 검증 설정, 실제 센서·위치, 동일 지도·시작 자세, 노트북의 완성된 계획 확인 후 PI 제어권 이양 |

기본 `command_timeout_s=0.30`은 **차량이 마지막으로 수락한 명령 이후** 기준이다. 브라우저는 0.10초마다 drive를 갱신한다. 최신 server_time이 0.30초보다 오래된 명령은 거부된다. 명령 하나가 늦었다고 WebSocket 세션을 해제하지는 않지만, 유효한 drive가 0.30초 동안 들어오지 않으면 차량이 정지하고 수동 재활성화가 필요하다. 이 값은 제어 응답과 정지시간 사이의 안전 한계이며 현재 Pi·Wi-Fi에서 실제 지연과 최악 정지시간을 계측해야 한다. 운영체제/커널 전체 정지, 전원 장애, 드라이버 고장까지 소프트웨어가 보장하지 않는다. 물리적 비상정지는 독립적으로 필요하다.

상태·지도 송신의 짧은 지연은 WebSocket 종료와 분리했다. 서버는 송신에 2초를 허용하고 WebSocket heartbeat를 6초로 설정했다. 실제 연결이 닫히면 브라우저가 재접속하며, 새 세션에서 제어권을 다시 획득해야 한다. 모터 출력은 위 0.30초 watchdog이 먼저 차단한다.

서보는 기본 정지 시 마지막 명령 위치를 유지하여 갑자기 가운데로 움직이지 않는다. `hold_on_stop=false`이면 펄스를 중지하나 하중에 의한 조향 이동이 가능하다. SIGKILL 시 커널 PWM이 남을 수 있고 컨테이너 정지 후 자동으로 센터가 되지는 않는다. 전체 전원 차단 정책을 실제 기계 상태에 맞게 검증한다. UI의 PWM은 **명령한 duty**이지 계측된 파형 값이 아니다.

## 7. ROS 및 WebSocket 인터페이스

| 인터페이스 | 자료형 / 내용 |
|---|---|
| `/rc/request` | std_msgs/String JSON; web_bridge → manager, 내부 session/수신시간 포함 |
| `/rc/command` | std_msgs/String JSON; 최종 제어 모드, lease, 제한, 목표, epoch |
| `/rc/state` | std_msgs/String JSON; 실제 카운트·RPM·거리·센서·명령 duty·상태·ROS stamp |
| `/rc/ack` | std_msgs/String JSON; 명령 수락/거부/동작 결과 |
| `/rc/action` | 내부 영점 동작 요청 |
| `/rc/odom_status` | std_msgs/String JSON; Odometry 유효성/사유 |
| `/rc/localization_health`, `/rc/parking_paths` | web_bridge → manager의 위치 품질·지도 버전, manager → web_bridge의 작업별 경로 |
| `/odom`, `/tf`, `/tf_static` | nav_msgs/Odometry 및 표준 TF |
| `/scan` | 기존 드라이버의 sensor_msgs/LaserScan |
| `/map` | SLAM Toolbox의 nav_msgs/OccupancyGrid |
| `/trailer/angle` | std_msgs/Float64; 영점/센서가 유효할 때만 rad 발행 |
| `/ultrasonic/us1` ... `/ultrasonic/us4` | sensor_msgs/Range, m, 실패는 NaN |

초음파 장착 TF는 실측 정보가 없어 생성하지 않는다. 향후 장애물 알고리즘은 센서 위치/방향 TF를 별도로 보정해야 한다. 현재 UI의 1~4번에 앞/뒤/좌/우라는 임의 위치를 붙이지 않는다. `/trailer/angle` 단독으로는 신선도를 판정하지 말고 `/rc/state`의 시간·유효성과 함께 사용한다.

WebSocket: `ws://Pi주소:8080/ws`, JSON 텍스트, 버전 `v=1`. 한 연결로 모든 명령/상태/지도 수신을 처리한다. `seq`는 연결별 증가 정수다. `server_time`은 **직전 state에 있는 값을 그대로** 사용한다. 노트북 시간으로 생성하지 않는다. session은 서버가 만든 제어권 라우팅 ID이며 클라이언트가 보내도 서버가 덮어쓴다. 계정/인증 토큰이 아니다. 다른 Origin은 거부하고 실제 접속 IP를 CIDR로 검사한다. reverse proxy 헤더를 신뢰하지 않는다.

순서:

1. `hello`에서 서버 상한과 session을 받는다. `state`에서 최신 시간과 센서 상태를 받는다.
2. `claim` 성공 응답을 기다린다.
3. `control`을 `PI` 또는 `OPEN_LOOP`로 설정한다. `limits`도 필요하면 설정한다.
4. `arm` 성공 응답을 기다린다. 0.3초 이내 drive를 보내기 시작한다.
5. 명령은 일반적으로 10Hz로 갱신한다. ACK를 확인하고 지연/오류 때 정지한다.

아래 값들은 **프로토콜 형식 예시**이며 실제 구동 설정으로 사용하지 않는다. `server_time`은 실제 받은 값으로 교체한다.

```json
{"v":1,"seq":1,"type":"claim","server_time":123.0}
{"v":1,"seq":2,"type":"control","control":"PI","server_time":123.1}
{"v":1,"seq":3,"type":"limits","pwm":0.1,"rpm":[20,20],"steering":0.1,"server_time":123.2}
{"v":1,"seq":4,"type":"arm","server_time":123.3}
{"v":1,"seq":5,"type":"drive","speed_m_s":0.05,"steering_rad":0.0,"server_time":123.4}
{"v":1,"seq":6,"type":"stop"}
{"v":1,"seq":7,"type":"release","server_time":123.5}
```

PI drive는 `speed_m_s` 또는 모터별 출력축 `rpm:[left,right]`를 받는다. 속도 명령은 휠베이스/윤거를 사용해 좌우 목표를 나눈다. 클램프 시 좌우 비율을 유지한다. Open-loop는 `pwm:[left,right]`이며 signed duty다. 두 방식 모두 `steering_rad`와 공통 manager/controller를 사용한다. UI는 RPM 목표를 모터별 제한×주행 목표 비율로 생성한다. 배열과 숫자는 길이·유한성 검사하며 NaN/Infinity/문자열 숫자/범위 위반은 거부한다.

기타 명령은 `estop`, `mode`, `zero_trailer`, `save_map` (`name`)이다. 수동 `arm`은 신선한 준비 상태 및 영출력에서 비상정지를 해제하고 새로운 epoch를 만든다. 주차 제어권 이양에는 아래의 별도 정지 상태 절차가 필요하다. ACK는 `{type:"ack",seq,ok,error?}`이며 drive ACK는 목표 수락을 표시한다.

`state`에는 `server_time`, `data.vehicle`, `data.command`, `data.odom`, `data.scan`, `data.odometry_status`, 각 `age_s`, `map_pose`, `slam_ready`가 있다. `map_pose`는 map 좌표계, `data.odom`은 odom 좌표계다. 지도는 `map_chunk` 여러 개로 전송한다. 각 조각은 공통 `version`, `width`, `height`, `resolution`, `origin[x,y,yaw]`, `offset`, `total`, `data`를 갖는다. data는 ROS와 같은 `row=y, column=x` 순서의 -1/0..100 배열이며, 브라우저와 노트북 클라이언트가 같은 버전의 조각을 모두 받은 뒤 사용한다. 지도 조각 사이에 상태와 ACK를 보내 큰 지도 전송 중 제어 지연을 줄인다. 지도 최대 크기와 전송 주기는 config에서 제한한다.

### 노트북에서 트레일러 주차 사용하기

**수동 조종은 브라우저만 열면 된다.** 자동주차 화면을 사용할 때는 노트북 터미널에서 Python 프로그램을 먼저 실행하고 그대로 둔다. 브라우저 버튼은 Pi에 계획이나 시작을 요청한다. Pi가 요청을 이미 연결된 노트북 프로그램에 전달한다. ROS, GPIO, SLAM Toolbox는 계속 Pi Docker에서 실행된다.

1. **실측 설정.** `config/vehicle.yaml`의 `geometry.wheelbase_m`, `servo.max_angle_rad`, `servo.slew_rad_s`를 확인한다. `parking`에는 뒷차축에서 히치까지의 후방 거리, 히치에서 트레일러 차축까지의 거리, 양 차체의 차축 기준 앞·뒤 돌출량 및 전체 폭을 m로 입력한다. 이 버전은 양 차축과 히치가 각각 차체 중심선에 있는 단일 트레일러를 대상으로 한다. 차량 `base_link` 원점은 뒤 차축 중심이어야 한다. MT6701의 부호는 **차량 yaw − 트레일러 yaw**이며 직진 영점과 좌우 회전을 실측한다.
2. **운영 한계.** 접힘·차체 간섭 전에 정지할 연결각, 허용 속도와 가속도, 위치 오차와 제동거리를 고려한 여유 거리·초음파 정지 거리, 허용 위치 분산·각도 분산, 출발 전 각 바퀴의 실제 에지를 얼마나 최근까지 인정할지 설정한다. `pos_tol_m`, `yaw_tol_rad`, `stop_speed_m_s`, `deviation_m`은 판정 기준이다. 실시간 계획에서는 Pi가 해당 설정을 노트북으로 보낸다. **현재는 `parking.commissioned: false`를 유지한다.**
3. **노트북 설치.** 프로젝트 루트에서 Python 3.10 이상으로 다음을 실행한다. macOS에서 실행 파일 이름이 `python3`이면 아래 명령의 `python`을 `python3`로 바꾼다. Pi 이미지에 이 수치 라이브러리를 설치할 필요는 없다.

```text
python -m pip install numpy scipy PyYAML Pillow aiohttp
python -m laptop.parking --help
```

#### A. 저장한 지도에서 경로만 계산

실제 저장한 Nav2 trinary 지도 YAML과 같은 폴더의 회색조 PGM 또는 기존 WebSocket `map` JSON을 노트북에 둔다. 아래는 명령 형식이다. 파일 경로와 대문자 인자는 측정한 값으로 바꾼다. `--start`는 차량 뒤 차축 중심과 차량 yaw, 연결각이다. `--goal`은 **트레일러 차축 중심**과 트레일러의 최종 앞 방향, 최종 연결각이다. 좌표는 map 기준 m, 각도는 rad이다.

```text
python -m laptop.parking offline --map maps/garage.yaml --start CAR_X CAR_Y CAR_YAW BETA --goal TRAILER_X TRAILER_Y TRAILER_YAW FINAL_BETA --initial-steer STEER_RAD --out parking-plan.json
```

이 명령은 Pi에 연결하지 않는다. 입력한 자세는 계획 조건으로만 사용한다. 출력 JSON의 `hybrid` 각 항목은 `[[x,y,yaw,beta], 부호있는_구간거리_m, 조향_rad]`, `reference`는 `[시간_s,[x,y,yaw,beta],속도_m_s,조향_rad]`이다. 충돌, 미설정 치수, 탐색 예산 초과 시 오류를 반환한다.

#### B. 웹 지도에서 계획 요청

Pi에 이 버전의 ROS 패키지·웹 파일·차량 설정을 반영하고 기존 배포 절차로 컨테이너를 실행한다. 아래 예시의 IP를 실제 Pi 주소로 바꾼다.

```text
python -m laptop.parking connect http://192.168.1.42:8080
```

같은 주소를 노트북 브라우저에서 열고 **제어권 획득 → 정지 → 지도에서 목표 지점을 누르고 트레일러 앞 방향으로 드래그 → 최종 연결각 입력 → 경로 계획** 순서로 사용한다. 비상정지 상태에서도 계획할 수 있다. 주황색은 Hybrid A*, 하늘색은 Planning MPC의 트레일러 차축 경로다. 라이다 구성은 유효한 라이다 Odometry와 map 자세, 지도, MT6701 영점·상대각, 실측 치수가 있어야 한다. 서보 명령이 없으면 정지 상태의 첫 조향을 중앙으로 가정한다. 지도만 있으면 A 절차를 사용한다.

#### C. 엔코더가 있는 기존 차량의 실차 검증 후 추종

현재 라이다 구성에서는 이 단계를 진행하지 않는다. 라이다가 추정한 차체 속도로 각 모터의 RPM을 측정할 수 없다. 이 구성에서 자동주차를 구동하려면 별도의 차체 속도 제어와 추정 지연·제동거리 검증이 필요하다.

9절의 실측 기록을 완료하고 승인한 설정에서만 관리자가 `commissioned`를 설정한다. 운영자는 정지한 차량에서 계획을 확인하고 **비상정지 해제 (정지 유지) → AUTO_PARK 시작**을 누른다. 양쪽 엔코더의 최근 실제 회전 이력과 정지 속도, PI·조향 상태, 영점이 잡힌 MT6701, 초음파 4개, 신선한 `/scan`·`/odom`·지도 TF와 허용 분산 내 `/pose`가 모두 필요하다. `/pose`가 설치한 SLAM 구성에서 발행되는지, 정지·저속에서도 주차에 필요한 빈도로 갱신되는지 확인한다. 서보 상태에는 보정된 **명령각**을 사용한다. 실제 조향 응답과 명령각 오차를 실측해야 한다. SLAM Toolbox는 처리한 스캔의 시각과 공분산을 자세 메시지에 기록한다. [SLAM Toolbox의 자세 발행 코드](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/src/slam_toolbox_common.cpp)

주행 중 목표 속도(m/s)·조향각(rad)은 노트북만 보내고, 두 모터의 PI와 서보는 공통 차량 제어기가 수행한다. 지도 내용 변경, 이탈, 연결각 초과, 초음파 접근·만료, 엔코더/위치 손실, 노트북·운영 브라우저 연결 종료 시 정지한다. 새로 제어권을 획득하고 현재 지도에서 다시 계획한다. **주차 중지**와 **비상정지**는 브라우저에서 즉시 요청할 수 있다.

| 단계 | 노트북 계산 |
|---|---|
| Trailer-aware Hybrid A* | 차량 뒤 차축의 자전거 모델과 오프축 히치의 트레일러 각속도를 부호 있는 거리로 적분한다. 후진 우선, 전진 보정·기어 전환 비용, 최종 후진 진입을 적용한다. 차량·연결부·트레일러 직사각형을 덮는 원들과 점유 격자 거리장을 사용해 이동 구간을 세분 검사한다. 미탐색·지도 밖은 장애물로 취급한다. |
| Planning MPC | 최대 5개 구간의 조향 후보를 반복 평가해 차체·트레일러 자세 오차, 조향 변화, 장애물 거리로 기준을 선택한다. 최종 자세를 벗어나면 검증된 원 경로를 사용한다. 조향·기어 변경 때 정차하도록 가감속 제한을 적용해 시간 기준 궤적을 만든다. |
| Tracking MPC | 최신 차량 자세·양쪽 바퀴 평균 속도·연결각·보정 조향 명령으로 5단계 속도·조향 후보를 예측하고 첫 명령만 적용한다. 이탈하거나 후보가 없으면 정지한다. 목표에서 양 차체의 위치·방향 허용 오차, 실측 정지 속도와 영출력을 확인해 완료한다. |

웹과 Python은 같은 `/ws`에 각각 한 번 연결한다. Python의 `park_register`·`park_ping`은 연결 상태만 갱신한다. 브라우저의 `park_plan`은 지도 버전·목표를 보내고, 노트북의 `park_result`는 작업 ID·시작 자세·두 경로를 돌려준다. Pi가 `parking_paths`와 `state.data.command.parking`을 브라우저로 전달한다. 자동 모드에서는 등록된 노트북의 현재 작업 ID가 있는 PI `drive`만 구동 명령으로 인정한다. `park_complete`, `park_abort`, 공통 `stop`·`estop`이 작업을 끝낸다. 재접속 후 자동 재출발하지 않는다.

## 8. 전원 재공급 후 운영

`deploy/run.sh`는 `--restart unless-stopped`로 컨테이너를 만든다. Docker 서비스가 부팅 시 시작되고 동일 gpiochip/PWM/USB가 존재하면 컨테이너가 다시 실행된다. 새 코드가 있으면 추가 패키지 빌드를 증분 수행한다. 하드웨어 초기화 실패 시 출력 준비가 되지 않았다고 표시한다. PWM 채널은 필요할 때 코드가 export하지만 올바른 overlay 및 chip mount가 먼저 존재해야 한다.

```bash
# Raspberry Pi OS
docker start rc-car             # 이전에 수동 stop 했다면 필요
docker logs --tail 100 rc-car
```

브라우저 접속 → 센서 상태 확인 → MT6701 영점 → 제어권 획득 → 수동 모드/제한 확인 → 활성화. 재시작만으로 차량은 움직이지 않는다. USB 재연결이나 장치 번호 변경 시 기존 Docker device mount가 유효한지 확인하고 컨테이너를 재생성한다. 자동주차는 여전히 잠겨 있다.

## 9. 실차 검증 체크리스트

아래는 **아직 미완료**인 인수 테스트다. 코드 테스트 통과로 대신하지 않는다.

- [ ] 전원·GND·3.3V GPIO 레벨·물리정지·전원 퓨즈·부팅 시 MDD 입력 LOW 확인
- [ ] 실제 기존 이미지 안 Jazzy/SLAM/드라이버 버전·패키지 확인, colcon/ROS 노드 실행
- [ ] Docker gpiochip 읽기/edge 알림, 서보 sysfs 쓰기, USB 단독 접근
- [ ] 각 모터 1회전/10회전 카운트, 정역 부호, 실제 감속비, RPM 기준 계측 비교
- [ ] 최대 예상 edge rate에서 카운트 손실과 CPU 부하 측정 (지도 전송/SLAM 동시 실행)
- [ ] 오실로스코프로 PWM 주파수/duty/변경 지연/finite train 종료 LOW, SIGSTOP/SIGKILL 후 잔류 펄스 확인
- [ ] 서보 펄스-타이어각 보정, 양 끝 기계 간섭, 전원 강하, 부하에서 실제 조향 지연 확인
- [ ] 낮은 출력 Open-loop → PI 순서로 검증, stall/엔코더 단선/반대 신호/과속 시 안전 동작
- [ ] MT6701 실제 PPR·ABZ·부호·직진 영점·Z 보정·전원 재공급 후 미영점 상태 확인
- [ ] US-100 4개 실제 거리 비교, 단선/timeout/stuck-high/상호 간섭 확인
- [ ] 브라우저 2개 동시 제어권 충돌, 탭 이동, Wi-Fi 단절, stale/replayed seq, 정지/재활성화 확인
- [ ] `/odom`, TF 직진/원호/후진 검증, 센서 유실 시 TF 발행 중단 및 재시작 필요 상태 확인
- [ ] 실제 지도 생성/차량 방향 표시/지도 저장/동일 시작 자세 localization 재사용
- [ ] 라이다 단독 구성에서 모터 전원을 끄고 직진·후진·회전·출발점 복귀 오차, 스캔 손실·복구, SLAM 동시 처리 주기와 조종 지연 측정
- [ ] 전원 재공급 후 비상정지 및 자동 미출력 확인
- [ ] 주차용 치수·연결각 부호·전후진 궤적을 실제 기록과 비교하고 차량/트레일러/히치의 여유 거리 및 간섭 한계 측정
- [ ] 실측 지도와 시작 자세로 경로 검토, 기록된 센서로 추종 오차·연산 시간 확인, 계측한 저속 정지거리로 초음파 한계·속도 상한 결정
- [ ] 모터 전력을 차단한 통합 시험에서 Wi-Fi 단절·운영자 종료·지도 갱신·접힘·피드백 손실 시 정지와 재출발 차단 확인

검증 기록은 이 README 아래에 날짜·설정값·계측 결과를 추가하면 된다. 확인하지 않은 항목을 체크하지 않는다.

## 10. 오류 해결 및 하드웨어 대안

| 증상 | 확인 및 조치 |
|---|---|
| 브라우저 403 | 실제 노트북 IP가 allowed_cidrs에 포함되는지 확인. Origin과 주소/포트가 같은지 확인 |
| 접속 실패 | Pi IP/동일 Wi-Fi/AP isolation/방화벽/8080 점유/docker logs 확인 |
| `hardware_initialization` / `GPIO busy` | gpiochip 번호와 line 점유, UART/SPI/1-wire/I2S, 중복 컨테이너 확인 |
| pwmchip/pwm1 없음 | GPIO13 overlay, 재부팅, npwm/channel 매핑, 실제 경로 bind mount 확인 |
| PWM 권한 오류 | 호스트 sysfs 및 mount RW 확인. 전체 privileged로 회피하지 말고 해당 장치 권한부터 확인 |
| 활성화 불가 | 모터 enabled, 전원/배선/정지 검증 플래그, 서보 보정·장치 초기화, 최신 state 확인 |
| PI 준비 안 됨 | 두 모터 encoder_enabled/verified/gear_ratio_verified/CPR/PI gain/최대 RPM 및 실제 edge 확인 |
| RPM/거리 30배 오류 | motor CPR와 output CPR 혼동 여부, gear_ratio 확인 |
| 카운트 방향 반대 | 모터 direction과 encoder_direction을 독립 검증. PI를 먼저 끄고 정지 |
| 엔코더 과다 오류/계측 불일치 | 배선 노이즈/전압/풀업/배선 길이/최대 이벤트율/CPU 지연 확인 |
| SLAM 실행 거부 | lidar 소스이면 C1 활성화, 실측 `xyz_rpy`, `mount_verified`, 컨테이너의 `rtabmap_odom` 패키지를 확인. wheel 소스이면 엔코더·조향·차량 치수 보정도 확인 |
| `/scan` 없음 | 기존 컨테이너 점유, USB/by-id, C1 460800, 전원/예열, 드라이버 로그 확인 |
| 지도는 있지만 위치 없음 | odom_status, TF 신선도, scan 신선도 확인. 화면이 자동으로 가짜 위치를 만들지 않음 |
| 지도 저장 오류 | use_map_saver 서비스, nav2_map_server, maps 쓰기권한, 이름 중복, 부분 파일 확인 |
| localization 위치 불일치 | 실제 차량이 map 시작/dock 자세인지 확인. 다른 자세 초기화는 아직 UI 미제공 |

Docker가 하드웨어 타이밍을 실시간으로 보장하지 않는다. 커널 PWM은 컨테이너 안에서 설정해도 호스트 PWM 주변장치가 출력한다. GPIO edge callback/PWM이 부하에서 불안정하면 운전을 중단한다. 다음은 **대안 설계이며 이 저장소에 구현된 다른 backend가 아니다**.

1. GPIO 제어 부분만 Pi OS의 작은 서비스로 이동하고 Docker `vehicle_controller`와 Unix-domain socket으로 연결할 수 있다. `hardware.py`의 실측/출력 코드를 재사용하되 sequence·lease·재접속 래치·정지 검증을 동일하게 구현해야 한다. ROS/SLAM/web은 Docker에 남겨두며 Pi OS에 ROS 전체를 중복 설치할 필요는 없다. 단순히 실행 위치만 옮겨도 Linux/Python의 비실시간 문제가 사라지는 것은 아니다.
2. 서보 sysfs만 Docker에서 접근이 안 되면 해당 PWM 설정만 호스트 helper로 옮길 수 있다. 무제한 네트워크 GPIO daemon을 Wi-Fi에 공개하지 않는다.
3. 최대 RPM에서 AB edge 손실이 발생하면 하드웨어 quadrature counter나 RP2040/STM32로 encoder/PWM/watchdog만 위임하는 것이 더 확실하다. 이 경우 firmware와 transport는 별도 구현·검증 대상이며 현재 완료된 것으로 표시하지 않는다.

## 11. 이번 Windows 검증 결과

- 제공 TXT 2개, PDF 4개 전체 텍스트, PNG 2개를 분석했다. MDD3A의 사양/진리표는 Windows PDF 렌더링으로 시각 확인했다.
- Python 파일 컴파일, JavaScript 구문 검사, Bash 구문 검사를 수행했다.
- 기존 제어 테스트와 주차 수학·통합 단위 테스트를 Windows에서 실행했다. 주차 테스트는 시험용 격자·자세로 후진·연결각·충돌·속도/조향 한계·목표 판정·작업별 제어권·피드백 유실·오프라인 CLI를 검사한다. 실제 주행 기록과 저장 지도는 이번 작업 폴더에 없어 실측 궤적 재검증은 남아 있다.
- preflight는 기본 설정 출력 차단을 확인했다. 현재 Windows에는 rclpy/lgpio/aiohttp와 Pi 장치가 없으므로 ROS/WebSocket 실제 서버 및 실차 통합 테스트를 실행하지 못했다.
- 브라우저 접근 호스트가 제공되지 않아 실제 브라우저 렌더링/키보드 E2E 검증도 미실행이다. 웹 UI가 동작 검증 완료라고 표시하지 않는다.

실차 측정과 검증 기록: **아직 없음**.
