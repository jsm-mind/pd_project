# 1호기 가정용 화재 대응 로봇 — 실행 가이드

단독주택을 순찰하다 화재를 발견하거나 화재 경보를 받으면 119 신고 → 거주자 탐색 → 대피유도 → 화재 진압을 하는 로봇의 Gazebo 시뮬레이션입니다.

- 브랜치: `rebuild-m1-m4` (현재 작업 브랜치)
- 개발·시연 환경: Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic (Jetson Orin Nano Super에서 확인)

아래 1~5단계를 순서대로 따라 하면 됩니다. 1~3단계는 처음 한 번만 하면 됩니다.

---

## 1. 필요한 프로그램 설치 (처음 한 번)

ROS 2 Humble과 Gazebo Harmonic이 이미 설치되어 있다고 가정합니다. 없으면 공식 문서대로 먼저 설치하세요.
- ROS 2 Humble: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- Gazebo Harmonic: https://gazebosim.org/docs/harmonic/install_ubuntu

그다음 나머지 패키지를 설치합니다.

```bash
sudo apt update
sudo apt install -y ros-humble-nav2-bringup ros-humble-navigation2 ros-humble-xacro \
    ros-humble-robot-state-publisher python3-gz-transport13 python3-gz-msgs10 \
    python3-numpy python3-yaml xterm pulseaudio-utils python3-colcon-common-extensions python3-rosdep
```

## 2. ros_gz 소스 빌드 (처음 한 번)

apt로 받는 `ros-humble-ros-gz`는 Gazebo Fortress용이라 Harmonic과 연결되지 않습니다. 그래서 `~/ros_gz_ws`에 직접 빌드합니다.

```bash
export GZ_VERSION=harmonic
mkdir -p ~/ros_gz_ws/src && cd ~/ros_gz_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b humble
cd ~/ros_gz_ws
sudo rosdep init 2>/dev/null; rosdep update
rosdep install -r --from-paths src -i -y --rosdistro humble
source /opt/ros/humble/setup.bash
colcon build --packages-up-to ros_gz_sim ros_gz_bridge
```

빌드는 Jetson 기준 수십 분 걸릴 수 있습니다.

## 3. 저장소 받기 + 환경 설정 (처음 한 번)

**반드시 `~/proj_ws` 위치에 받아 주세요.** 일부 설정 파일 경로가 `~/proj_ws` 기준입니다.

```bash
cd ~
git clone -b rebuild-m1-m4 https://github.com/jsm-mind/pd_project.git proj_ws
```

`~/.bashrc` 맨 끝에 아래를 추가합니다.

```bash
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
source ~/ros_gz_ws/install/setup.bash
export GZ_IP=127.0.0.1
[ -f ~/proj_ws/install/setup.bash ] && source ~/proj_ws/install/setup.bash
alias fire='~/proj_ws/run.sh'
```

적용합니다.

```bash
source ~/.bashrc
```

## 4. 빌드

```bash
cd ~/proj_ws
colcon build
source install/setup.bash
```

코드를 새로 받았을 때(`git pull`)는 다시 빌드해야 합니다. `fire build`로 실행하면 빌드 후 바로 실행됩니다.

## 5. 실행

터미널 하나에서 아래 명령 하나면 전부 뜹니다. Gazebo, 로봇, 모드 관리, 감지, 연락, Nav2, 순찰, 키 입력 창이 모두 실행됩니다.

```bash
fire
```

- 약 15초 뒤 Nav2가 뜨면 로봇이 **자동으로 순찰을 시작**합니다.
- 순찰 시작 후 10~40초 사이에 **랜덤한 위치에서 불이 납니다.**
- 이후 시나리오는 모두 자동으로 진행됩니다.

### 실행 옵션

옵션은 자유롭게 조합할 수 있습니다. 예: `fire build noalarm stove_1`

| 옵션 | 설명 |
| --- | --- |
| (없음) | 랜덤 위치에 불, 경보기 정상 |
| 불 위치 | `stove_1` `stove_2` `tv_stand` `nightstand_bedroom1` `nightstand_bedroom2` `utility_room` 중 하나 |
| `noalarm` | 화재 경보기 고장. 로봇이 순찰 중 불을 직접 찾아야 함 (시나리오 1만 나옴) |
| `build` | 실행 전에 `colcon build` |
| `rviz` | RViz도 같이 띄움 (맵, 로봇 위치, 경로) |
| `call` | 119 대신 지정 번호로 **실제 전화**를 검 (아래 '119 실제 전화' 참고). 없으면 로그만 출력 |

### 키 입력 창 (같이 뜨는 작은 xterm 창)

| 키 | 동작 |
| --- | --- |
| `p` | 순찰 시작 |
| `c` | 충전기로 복귀 |
| `a` | 화재 경보 울리기 (불이 아직 안 났으면 바로 남). `noalarm`일 때는 경보 없이 불만 남 |
| `r` | 긴급모드 해제 → 충전기로 복귀 |
| `q` | 키 창 종료 |

### 종료

`fire`를 실행한 터미널에서 `Ctrl+C`. 에어로졸은 1회용이라, 다음 시연은 `fire`를 다시 실행하세요.

---

## 시나리오별 시연 방법

| 보고 싶은 것 | 명령 |
| --- | --- |
| 시나리오 1: 순찰 중 불 발견 | `fire noalarm` (불 위치를 정하려면 `fire noalarm stove_1`) |
| 시나리오 2: 화재 경보 수신 | `fire` 실행 후 불이 나기 전에 키 창에서 `a` |
| 시나리오 3: 충전 ↔ 순찰 | 순찰 중 `c` → 충전기 도착 후 `p` |

그냥 `fire`로 실행하면, 불이 난 뒤 10초 안에 로봇이 불을 보면 시나리오 1, 못 보면 경보를 받아 시나리오 2로 진행됩니다.

### 화면에서 보이는 것

- **상태 링** (로봇 윗면): 파랑 = 순찰 / 흰색 = 충전기 복귀·충전 / 빨강 빠른 점멸 = 119 신고·연락 / 빨강 = 거주자 탐색·화재 위치로 이동 / 초록 점멸 = 대피유도 / 보라 점멸 = 에어로졸 방출 / 보라 = 진압 후 연락
- **소리**: 긴급모드 진입 시 사이렌, 불이 난 뒤 5초부터 화재 경보음, 로봇 음성 안내
- **보호자/거주자 연락**: `fire`를 실행한 터미널에 출력됨

---

## 소방서 관제 중계 화면 (선택)

같은 네트워크의 다른 기기(노트북, 휴대폰) 브라우저에서 엽니다.

```
http://<시뮬 돌리는 PC의 IP>:8080
```

IP는 `hostname -I`로 확인합니다. 영상(열화상)과 위치는 긴급모드일 때만 보이고, 평소에는 '대기'로 표시됩니다.

## 119 실제 전화 (선택)

Twilio 계정이 있어야 합니다. `~/proj_ws/twilio.yaml`을 만들고 아래처럼 채웁니다. 이 파일은 git에 올라가지 않습니다.

```yaml
contact:
  ros__parameters:
    twilio_sid: "AC..."
    twilio_token: "..."
    twilio_from: "+1..."          # Twilio 발신 번호
    call_number: "+8210..."       # 119 대신 걸 번호 (Twilio에 인증된 번호)
```

그다음 `fire call`로 실행합니다. 파일이 없거나 `call`을 빼면 전화는 걸지 않고 로그만 남깁니다.

---

## 문제 해결

| 증상 | 해결 |
| --- | --- |
| 로봇이 안 보임 | 이전에 띄운 Gazebo나 노드가 남아 있는 경우가 대부분입니다. 모두 끄고 다시 실행하세요: `pkill -f "gz sim"; pkill -f ros2` (다른 터미널에서 실행) |
| `fire: command not found` | 3단계의 `alias` 추가 후 `source ~/.bashrc`를 했는지 확인. 또는 `~/proj_ws/run.sh`로 직접 실행 |
| `package 'ros_gz_sim' not found` | 2단계 빌드 후 `source ~/ros_gz_ws/install/setup.bash`가 `.bashrc`에 있는지 확인 |
| 소리가 안 남 | 출력 장치를 확인하세요. HDMI로 내보내려면 `pactl list short sinks`로 이름을 확인하고 `pactl set-default-sink <이름>` |
| 순찰을 시작하지 않음 | Nav2가 뜨는 데 시간이 걸립니다. 30초쯤 기다리거나 키 창에서 `p` |
| 같은 네트워크 다른 PC의 시뮬과 섞임 | `~/.bashrc`에 `export ROS_DOMAIN_ID=<팀원마다 다른 숫자>`, `export GZ_PARTITION=<내 이름>` 추가 |

## 더 자세한 정보

- 명령어 모음, 단독 실행 방법, 테스트용 토픽: [실행 방법.txt](실행%20방법.txt)
- 프로젝트 사양·마일스톤: [CLAUDE.md](CLAUDE.md)
