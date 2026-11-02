# 1호기 차체 인터페이스 인벤토리

차체(로봇 모델)를 새 디자인으로 교체하기 전에, 현재 임시 차체에 연결된 파일·센서·토픽·프레임·파라미터를 조사한 결과다.

- 조사 기준: `src/` (2026-10-29 기준)
- `pd_project/src/`는 `src/`와 내용이 같은 복사본이다 (`__pycache__` 제외, 미추적 상태). 아래 경로는 모두 `src/` 기준이다.
- `unused/`는 이번 시뮬레이션 패키지와 무관한 코드라서 조사하지 않았다.

---

## 1. 차체 모델 관련 파일

| 구분 | 파일 | 역할 |
|---|---|---|
| URDF/xacro | `fire_robot_description/urdf/fire_robot.urdf.xacro` | 차체·바퀴·캐스터·센서·노즐 링크, Gazebo 센서, DiffDrive/JointStatePublisher 플러그인 **(유일한 차체 정의)** |
| launch | `fire_robot_description/launch/description.launch.py` | xacro → `robot_description`, `robot_state_publisher` 실행 |
| launch | `fire_robot_gazebo/launch/sim.launch.py` | Gazebo 실행, 로봇 spawn(`-name fire_robot`, x 4.5 / y 2.0 / z 0.01 / yaw 0), 브리지 2개 |
| launch | `fire_mission/launch/full.launch.py` | 전체 시나리오. 로봇을 **따로 spawn** (같은 이름·좌표가 하드코딩됨) |
| 브리지 설정 | `fire_robot_gazebo/config/bridge.yaml` | 로봇 센서·구동 토픽 브리지 |
| 브리지 (인라인) | `sim.launch.py` 의 `sim_tools_bridge` | spawn/remove 서비스, ground-truth pose |
| world | `fire_robot_gazebo/worlds/house.sdf` | 집 월드. 로봇은 포함하지 않음. Sensors(ogre2)·ParticleEmitter 시스템, 대기 온도 293 K, 물리 스텝 0.004 s |
| 분사 모델 | `fire_mission/models/spray.sdf` | 분사 파티클 모델 (차체와 별도) |
| 화재 모델 | `fire_sim/models/fire.sdf` | 반경 0.15 m, 높이 0.4 m 원기둥, 600 K |
| Nav2 설정 | `fire_robot_navigation/config/nav2_params.yaml` | 프레임, 로봇 반경, 속도 한계 |
| 맵 | `fire_robot_navigation/maps/house.{pgm,yaml}` | 해상도 0.05 m, origin (-0.55, -0.55) |
| 맵 생성 스크립트 | `fire_robot_navigation/scripts/world_to_map.py` | world의 박스 충돌체를 래스터화 (높이 0.03 m 미만 박스 제외) |
| 커버리지 검증 | `fire_mission/scripts/check_coverage.py` | 웨이포인트 열화상 커버리지 오프라인 검증 (`MAX_RANGE = 6.0` m) |
| 실행 메모 | `실행 방법.txt` | 수동 실행 순서 |

---

## 2. 차체 형상과 센서 사양

### 2.1 차체 (xacro 프로퍼티)

`base_link` 원점은 바퀴 축 바로 아래의 지면이다. `base_footprint`와 `base_link`는 오프셋 없는 고정 조인트로 연결되어 있다.

| 항목 | 값 |
|---|---|
| 본체 박스 | 0.30 (x) × 0.24 (y) × 0.10 (z) m, 지상고 0.03 m → 본체 z 0.03–0.13 m |
| 본체 질량 | 2.0 kg |
| 구동 바퀴 | 반경 0.05 m, 폭 0.03 m, 0.2 kg, 좌우 간격 0.28 m, 조인트 위치 (0, ±0.14, 0.05), 축 +y, μ = 1.0 |
| 캐스터 | 구 반경 0.025 m, 전후 2개, x = ±0.11, 마찰 0 (고정 조인트) |
| 외곽 | x −0.15 ~ +0.15, y −0.155 ~ +0.155 (바퀴 포함) → 원점 기준 최대 반경 약 0.20 m |

### 2.2 센서

| 항목 | 2D LiDAR | 열화상 카메라 |
|---|---|---|
| Gazebo sensor | `lidar` (`gpu_lidar`) | `thermal_camera` (`thermal`) |
| 링크 / frame_id | `lidar_link` (`gz_frame_id`) | `thermal_link` (`gz_frame_id`) |
| 장착 위치 (`base_link` 기준) | (0.05, 0, 0.155) | (0.05, 0, 0.20), LiDAR 바로 위 |
| 방향 | rpy 0 (전방 +x) | rpy 0 (+x 방향을 봄) |
| 화각 | 수평 360° (−3.14159 ~ 3.12414 rad) | 수평 1.0472 rad (60°), 수직은 종횡비로 약 46.8° |
| 해상도 | 360 샘플 (1°), 거리 분해능 0.01 m | 160 × 120, `L16` (픽셀값 = 온도 / 0.01 K) → ROS `mono16` |
| 측정 범위 | 0.12 – 8.0 m, 가우시안 노이즈 σ 0.01 m | clip 0.1 – 10.0 m. 실제 감지 거리는 6 m까지 검증됨 (`check_coverage.py`) |
| 갱신 주기 | 10 Hz | 10 Hz |
| 외형 | 원기둥 r 0.03, h 0.04 (visual만, z 0.135–0.175) | 박스 0.04 × 0.06 × 0.05 (visual만, z 0.175–0.225) |

RGB 카메라, IMU, 깊이 센서는 없다.

### 2.3 기타 링크

| 링크 | 위치 (`base_link` 기준) | 비고 |
|---|---|---|
| `nozzle_link` | (0.17, 0, 0.11), 전방 +x | 원기둥 r 0.015, 길이 0.04 (visual만). 끝단 x = 0.19 |
| `caster_link`, `front_caster_link` | (∓0.11, 0, 0.025) | |
| `left_wheel_link`, `right_wheel_link` | (0, ±0.14, 0.05) | continuous 조인트 |

---

## 3. 토픽 · 메시지 타입 · TF

### 3.1 차체가 내보내거나 받는 토픽 (`bridge.yaml`)

| ROS 토픽 | ROS 타입 | 방향 | Gazebo 쪽 출처 | frame |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | GZ→ROS | world | – |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS→GZ | DiffDrive 입력 | – |
| `/odom` | `nav_msgs/msg/Odometry` | GZ→ROS | DiffDrive, 20 Hz | `odom` → `base_footprint` |
| `/tf` | `tf2_msgs/msg/TFMessage` | GZ→ROS | DiffDrive (`odom`→`base_footprint`) | |
| `/joint_states` | `sensor_msgs/msg/JointState` | GZ→ROS | JointStatePublisher (`left_wheel_joint`, `right_wheel_joint`) | |
| `/scan` | `sensor_msgs/msg/LaserScan` | GZ→ROS | `lidar` | `lidar_link` |
| `/thermal/image` | `sensor_msgs/msg/Image` (`mono16`) | GZ→ROS | `thermal_camera` | `thermal_link` |

### 3.2 시뮬레이션 전용 브리지 (`sim_tools_bridge`)

| ROS 이름 | 타입 | 사용처 |
|---|---|---|
| `/world/house/create` | `ros_gz_interfaces/srv/SpawnEntity` | fire_generator, spray_nozzle |
| `/world/house/remove` | `ros_gz_interfaces/srv/DeleteEntity` | fire_manager, spray_nozzle |
| `/sim/ground_truth_poses` (← `/world/house/dynamic_pose/info`) | `tf2_msgs/msg/TFMessage` | fire_manager, spray_nozzle. `child_frame_id == 'fire_robot'`(모델 이름)인 항목만 사용 |

### 3.3 TF 트리

```
map ──(AMCL)──> odom ──(Gazebo DiffDrive, /tf)──> base_footprint ──(fixed, 0 오프셋)──> base_link
                                                                                         ├─ left_wheel_link / right_wheel_link  (RSP + /joint_states)
                                                                                         ├─ caster_link / front_caster_link     (fixed)
                                                                                         ├─ lidar_link                          (fixed)
                                                                                         ├─ thermal_link                        (fixed)
                                                                                         └─ nozzle_link                         (fixed)
```

- `base_footprint` 아래는 `robot_state_publisher`가 xacro로부터 발행한다.
- Gazebo 모델 원점 = URDF 루트 = `base_footprint`. ground-truth pose도 이 원점 기준이다.

---

## 4. 구동 방식

| 항목 | 값 | 위치 |
|---|---|---|
| 방식 | 차동 구동 2륜 + 무마찰 구형 캐스터 2개 | xacro |
| 플러그인 | `gz-sim-diff-drive-system` (`gz::sim::systems::DiffDrive`) | xacro |
| 조인트 | `left_wheel_joint`, `right_wheel_joint` | xacro |
| `wheel_separation` / `wheel_radius` | 0.28 / 0.05 m | xacro |
| 가속 한계 | 선 1.0 m/s², 각 3.0 rad/s² | xacro |
| 토픽 | `topic /cmd_vel`, `odom_topic /odom`, `tf_topic /tf` | xacro |
| 프레임 | `frame_id odom`, `child_frame_id base_footprint` | xacro |
| odom 주기 | 20 Hz | xacro |
| 조인트 상태 | `gz-sim-joint-state-publisher-system` → `/joint_states` | xacro |

`/cmd_vel`을 발행하는 쪽:
- Nav2 `velocity_smoother` (Humble `navigation_launch.py`: controller → `cmd_vel_nav` → smoother → `/cmd_vel`). 한계 `[0.4, 0, 1.5]`, 가속 `[1.0, 0, 3.0]`
- Nav2 `behavior_server` (spin): 0.4–0.6 rad/s
- `mission_node`: 각속도만 발행 (ALIGN 최대 0.4 rad/s, VERIFY 0.6 rad/s)

---

## 5. 분사 효과

분사 파티클은 **차체 URDF에 붙어 있지 않다.** 별도 모델을 노즐 위치에 spawn/remove 하는 방식이다 (xacro 214–215행 주석 참고).

| 항목 | 내용 |
|---|---|
| 노드 | `fire_mission/spray_nozzle.py` (`spray_nozzle`) |
| 트리거 | `/spray/active` (`std_msgs/Bool`). `true`가 0.5 s 안에 다시 오지 않으면 끈다 |
| 표시 | `models/spray.sdf` (정적 모델 `spray`, point `particle_emitter`, +x 방향 분사, rate 30, lifetime 0.6 s, 속도 2.5–3.0 m/s, 크기 0.06 m)를 `/world/house/create`로 spawn, 끌 때 `/world/house/remove` |
| 위치 계산 | `/sim/ground_truth_poses`에서 모델 `fire_robot`의 (x, y, z, yaw)를 읽고, 모델 원점 기준 오프셋을 더한다 |
| 오프셋 파라미터 | `nozzle_x = 0.19` (노즐 끝), `nozzle_z = 0.11`, `nozzle_pitch = −0.08` rad (약간 아래) — **코드 기본값, yaml 없음** |
| 가정 | 노즐이 차체 중심선(y = 0) 위에 있고, 차체가 수평이며(roll/pitch 무시), 분사 중 로봇이 정지해 있음 |
| world 요구 | `gz-sim-particle-emitter-system` 플러그인 (`house.sdf`) |

URDF의 `nozzle_link`는 외형과 TF용일 뿐이며, spray_nozzle은 이 링크를 TF로 읽지 않는다. 노즐 위치를 바꾸면 xacro와 spray_nozzle 파라미터를 **둘 다** 맞춰야 한다.

---

## 6. 행동 로직 노드별 의존성

### 6.1 `fire_detector` (`fire_detection`)

| 구분 | 내용 |
|---|---|
| 구독 | `/thermal/image` (`sensor_msgs/Image`, `mono16` 아니면 무시) |
| 발행 | `/fire_detection/detected` (`std_msgs/Bool`, 매 프레임), `/fire_detection/bearing` (`std_msgs/Float32`, rad, 왼쪽 +) |
| 파라미터 (`config/fire_detection.yaml`) | `threshold_k 450`, `resolution_k 0.01`, `min_pixels 4`, `confirm_frames 3`, `clear_frames 3`, **`hfov 1.0472` (URDF와 같아야 함)** |
| 차체 의존 | 방위각은 카메라 광축 기준이다. 해상도·hfov·L16 분해능이 바뀌면 파라미터도 바뀌어야 한다. 프레임 이름은 사용하지 않는다 |

### 6.2 `mission_node` (`fire_mission`, 순찰·신고·추적·정렬·분사·확인 상태 머신)

| 구분 | 내용 |
|---|---|
| 구독 | `/fire_detection/detected`, `/fire_detection/bearing`, `/scan` (sensor QoS), `/map` (transient local) |
| 발행 | `/cmd_vel` (각속도만), `/spray/active`, `/patrol/started` (`std_msgs/Empty`, latched) |
| TF | `map` → `base_link` lookup (AMCL 위치) |
| Nav2 | `BasicNavigator`: `goToPose`, `spin(2π)`, `amcl`·`bt_navigator` 활성화 대기 |
| 파라미터 (`config/mission.yaml`) | `waypoints` 7개, `spin_time_allowance 30`, `room_names/room_bounds`, `spray_distance 1.5`, `min/max_distance 1.0/2.0`, `fire_radius 0.15`, `align_tolerance_deg 3`, `realign_deg 8`, `align_gain 1.5`, `max_align_speed 0.4`, `align_timeout 20`, `spray_duration 12`, `scan_speed 0.6`, `verify_timeout 30`, `max_retries 3` |
| **센서 위치 (TF)** | 시작 시 TF `base_link → thermal_link`, `base_link → lidar_link`의 (x, y)를 1회 읽음 (2026-10-29 변경, 이전에는 코드 상수 `SENSOR_X = 0.05`). 코드 상수: `RANGE_WINDOW 2°`, `STATIC_MARGIN 0.15 m` |
| 숨은 가정 | ① 각 LiDAR 반사점의 방위각을 열화상 카메라 위치에서 다시 계산해 열화상 방위각과 비교 → 두 센서 위치가 달라도 됨. 단, 두 센서 모두 yaw 0 ② LaserScan 각도 0 = 차체 전방 ③ 거리 판정 = 추정 화재 중심(카메라 + (LiDAR 거리 + `fire_radius`) × 방위각 방향)과 `base_link` 사이 거리 ④ 정렬 = 차체 전방(= 카메라 광축 = 노즐 방향) → **열화상 카메라는 중심선(y = 0) 위에 있어야 함** |
| 신고 | 터미널 `print` 1회 (시각, "화재 감지 — 119 신고", 방 이름 + 맵 좌표) |

### 6.3 `spray_nozzle` (`fire_mission`) — 5장 참고

구독 `/spray/active`, `/sim/ground_truth_poses` / 서비스 `/world/house/create`, `/world/house/remove` / 파라미터 `world_name house`, `robot_name fire_robot`, `nozzle_x 0.19`, `nozzle_z 0.11`, `nozzle_pitch −0.08`.

### 6.4 `fire_generator` (`fire_sim`)

구독 `/patrol/started`, `/fire_sim/extinguished` / 발행 `/fire_sim/spawned` (`geometry_msgs/PointStamped`, frame `map`, latched) / 서비스 `/world/house/create` / 파라미터 `candidates` 10곳, `min/max_delay`, `wait_for_patrol`. 차체 의존 없음 (후보 위치는 벽·가구 기준).

### 6.5 `fire_manager` (`fire_sim`, 분사 유효 판정)

| 구분 | 내용 |
|---|---|
| 구독 | `/fire_sim/spawned`, `/sim/ground_truth_poses`, `/spray/active` |
| 발행 | `/fire_sim/hp` (`Float32`), `/fire_sim/extinguished` (`Empty`) |
| 서비스 | `/world/house/remove` |
| 파라미터 (`config/fire_sim.yaml`) | `robot_name fire_robot`, `max_hp 100`, `damage_rate 10`, `min/max_distance 1.0/2.0`, `max_angle_deg 10` |
| 차체 의존 | 거리·각도를 **모델 원점(`base_footprint`)과 모델 yaw** 기준으로 계산. 원점이 바퀴 축 위가 아니면 판정 기준점이 바뀐다 |

### 6.6 Nav2 (`fire_robot_navigation`)

| 노드 | 사용 프레임 | 사용 토픽 |
|---|---|---|
| `amcl` | `base_frame_id: base_footprint`, `odom`, `map` | `scan` (laser 0.12–8.0 m, `DifferentialMotionModel`) |
| `bt_navigator` | `map`, `base_link` | `/odom` |
| `local_costmap` | `odom`, `base_link` | `/scan` |
| `global_costmap` | `map`, `base_link` | `/scan`, `/map` |
| `behavior_server` | `odom`, `base_link` | `local_costmap/costmap_raw`, `local_costmap/published_footprint` |
| `velocity_smoother` | – | `odom` |

---

## 7. 차체 크기에 의존하는 설정

| 설정 | 현재 값 | 파일 | 근거 / 주의 |
|---|---|---|---|
| `robot_radius` (local·global costmap) | 0.2 m | `nav2_params.yaml` | 외곽 최대 반경 약 0.20 m. footprint 다각형은 쓰지 않음 |
| `inflation_radius` | 0.35 m | `nav2_params.yaml` | 방문 폭 1.0 m (방1 x 2.5–3.5, 방2 x 7–8) 통과와 연관 |
| local costmap 크기 | 3 × 3 m | `nav2_params.yaml` | |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.15 m / 0.2 rad | `nav2_params.yaml` | |
| RPP 속도 | `desired_linear_vel 0.4`, `rotate_to_heading_angular_vel 1.5`, `max_angular_accel 3.2`, `allow_reversing false` | `nav2_params.yaml` | 차동 구동 전제 (제자리 회전) |
| spin 속도 | 0.4–0.6 rad/s | `nav2_params.yaml` | 열화상 60° 화각에서 화재가 약 17프레임 머물도록 정한 값 → **hfov와 연동** |
| `velocity_smoother` 한계 | `[0.4, 0, 1.5]`, 가속 `[1.0, 0, 3.0]` | `nav2_params.yaml` | DiffDrive 가속 한계(1.0 / 3.0)와 일치 |
| AMCL `laser_min_range` / `laser_max_range` | 0.12 / 8.0 m | `nav2_params.yaml` | LiDAR 사양과 일치 |
| AMCL·spawn 초기 위치 | (4.5, 2.0, yaw 0) | `nav2_params.yaml`, `sim.launch.py`, `full.launch.py` | 세 곳에 중복 |
| spawn 높이 | z 0.01 m | `sim.launch.py`, `full.launch.py` | 바닥이 원점인 차체 기준 |
| 센서 위치 | TF에서 읽음 | `mission_node.py` | 차체 xacro만 바꾸면 됨 (이전 `SENSOR_X` 상수는 삭제) |
| `nozzle_x` / `nozzle_z` / `nozzle_pitch` | 0.19 / 0.11 / −0.08 | `spray_nozzle.py` (기본값) | 노즐 위치 |
| `hfov` | 1.0472 rad | `fire_detection.yaml` | 열화상 화각 |
| 웨이포인트 | 7개 | `mission.yaml` | 로봇 반경 + 인플레이션으로 도달 가능한 위치, 열화상 높이 기준 커버리지 검증됨 |
| `check_coverage.py` | `MAX_RANGE 6.0` m, "가구가 열화상 카메라보다 높다"는 가정 | 스크립트 | 카메라 높이(0.20 m)·해상도가 바뀌면 재검증 필요 |
| LiDAR 높이 vs 맵 | 스캔 평면 0.155 m | xacro ↔ `world_to_map.py` | 맵은 높이 0.03 m 이상 박스를 모두 장애물로 그림. 가장 낮은 가구 상단은 0.45 m (커피 테이블), 침대·TV장 0.5 m. **스캔 평면이 0.45 m보다 높아지면 맵과 LiDAR 관측이 어긋나 AMCL이 흔들린다** |
| 열화상 높이 vs 화재 | 카메라 0.20 m, 화재 0–0.4 m | xacro ↔ `fire.sdf` | 수직 화각 약 47°. 카메라가 높아지면 1.0 m 근거리에서 화재가 화면 아래로 빠질 수 있음 |
| 자기 가림 | LiDAR(0.135–0.175 m)가 본체 상단(0.13 m) 위, 열화상(0.175 m~)이 LiDAR 위 | xacro | 새 차체에서 LiDAR 스캔 평면을 차체·마스트·노즐이 가리면 안 됨 (min range 0.12 m) |
| 물리 스텝 | 0.004 s | `house.sdf` | 무한궤도 등 복잡한 하부로 바꾸면 재검토 대상 |

---

## 8. 차체를 새로 만들 때 반드시 유지해야 하는 인터페이스

상위 로직(순찰·감지·추적·분사·화재 시뮬레이션)과 Nav2 설정을 고치지 않고 차체만 교체하려면 아래를 지켜야 한다. 어긋나는 항목은 괄호 안 파일을 함께 수정해야 한다.

**모델 / 프레임**
1. Gazebo 모델 이름 `fire_robot` (spawn 인자, `robot_name` 파라미터, ground-truth 필터)
2. 링크 이름 `base_footprint`(URDF 루트, 지면), `base_link`, `lidar_link`, `thermal_link`, `nozzle_link`
3. `base_footprint` = Gazebo 모델 원점 = 회전 중심. `base_link`와 오프셋 0 (AMCL은 `base_footprint`, 나머지는 `base_link`를 쓰기 때문)
4. TF 체인 `odom → base_footprint`를 시뮬레이터가 `/tf`로 발행, `base_footprint` 이하는 `robot_state_publisher`가 `robot_description`으로 발행

**구동**
5. `/cmd_vel` (`geometry_msgs/Twist`) 입력, `linear.x` + `angular.z`로 **제자리 회전 가능** (spin 스캔, ALIGN, VERIFY가 제자리 회전에 의존)
6. `/odom` (`nav_msgs/Odometry`, `odom` → `base_footprint`) 발행
7. 속도·가속 성능: 선속도 0.4 m/s, 각속도 1.5 rad/s 이상, 가속 1.0 m/s² / 3.0 rad/s² 수준 (Nav2·smoother 한계)
8. `/clock`, `/joint_states` 브리지 유지 (하부 조인트가 바뀌면 JointStatePublisher 조인트 목록과 RSP용 조인트를 맞출 것)

**LiDAR**
9. 토픽 `/scan` (`sensor_msgs/LaserScan`), frame `lidar_link`, 360°, 각도 0 = 차체 전방
10. 범위 0.12–8.0 m, 10 Hz (AMCL `laser_min/max_range`)
11. 위치: yaw 0 (x·y는 TF로 읽으므로 자유). 스캔 평면은 가장 낮은 가구(0.45 m)보다 낮고, 차체가 스캔을 가리지 않을 것

**열화상 카메라**
12. 토픽 `/thermal/image` (`sensor_msgs/Image`, `mono16`, 0.01 K/단위), frame `thermal_link`
13. 수평 화각 1.0472 rad (`fire_detection.yaml`의 `hfov`), 160 × 120, 10 Hz (spin 속도·`min_pixels`·`confirm_frames`가 이 값 기준)
14. 위치: 중심선(y = 0), yaw 0(광축 = 차체 전방). x는 TF로 읽으므로 자유. 높이는 0.4 m 화재를 1–6 m 거리에서 볼 수 있을 것. 렌즈가 차체 밖에 있어 자기 몸체가 시야에 들어오지 않을 것 (Gazebo thermal 센서는 near clip이 자기 몸체를 가려 주지 않음)

**노즐 / 분사**
15. 노즐이 중심선 위에서 차체 전방(+x)을 향할 것 (정렬 = 차체 heading 기준)
16. 노즐 끝 위치 = 모델 원점 기준 (0.19, 0, 0.11), 피치 −0.08 rad (`spray_nozzle` 파라미터)
17. `/spray/active` (`std_msgs/Bool`) 의미 유지, world에 ParticleEmitter 시스템 유지

**크기**
18. 원점 기준 외곽 반경 0.2 m 이내 (`robot_radius`). 넘으면 `robot_radius`·`inflation_radius`·웨이포인트를 재검토하고 방문(1.0 m) 통과를 확인
19. 바닥 기준 원점이라 spawn z = 0.01 m 유지 가능해야 함 (아니면 `sim.launch.py`·`full.launch.py` 수정)

**값이 여러 곳에 중복되어 있어 함께 바꿔야 하는 항목**

| 값 | 위치 |
|---|---|
| 열화상 hfov 1.0472 | xacro `thermal_hfov`, `fire_detection.yaml` `hfov` |
| 노즐 위치 | xacro `nozzle_x`·`nozzle_z` (0.17 / 0.11, 링크 중심), `spray_nozzle.py` `nozzle_x`·`nozzle_z` (0.19 / 0.11, 끝단) |
| 모델 이름 `fire_robot` | `sim.launch.py`, `full.launch.py`, `fire_sim.yaml`, `spray_nozzle.py` 기본값 |
| spawn 위치 (4.5, 2.0, 0) | `sim.launch.py`, `full.launch.py`, `nav2_params.yaml` `initial_pose` |
| LiDAR 범위 0.12–8.0 | xacro, `nav2_params.yaml` AMCL |
| 가속 한계 1.0 / 3.0 | xacro DiffDrive, `nav2_params.yaml` `velocity_smoother` |
