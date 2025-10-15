# Multi-Robot NS-3 Network Simulation with Dual Network Architecture

이 가이드는 이중 네트워크 구조를 사용하여 멀티 로봇 시뮬레이션을 실행하는 방법을 설명합니다.

## 시스템 아키텍처

### 이중 네트워크 구조 (Dual Network Architecture)

시스템은 **두 개의 독립적인 네트워크**를 사용합니다:

```
┌─────────────────────────────────────────────────────────────────┐
│                   Network Namespace (nns1, nns2, ...)            │
│                                                                   │
│  1️⃣ WiFi Network (10.0.0.0/9)                                    │
│     - NS-3 WiFi 시뮬레이션을 통과                                  │
│     - 로봇 간 ROS2 통신 (센서 데이터, 협업 명령)                     │
│     - 패킷 손실, RSSI, 지연의 영향을 받음                           │
│                                                                   │
│  2️⃣ Direct Network (10.128.x.x/29)                               │
│     - veth pair로 호스트와 직접 연결                               │
│     - 로봇 제어 명령 (cmd_vel) 전송                                │
│     - NS-3 우회, 지연 없음                                         │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
                          ↓                        ↓
                   NS-3 WiFi 시뮬레이션      ROS2-Gazebo Bridge
                          ↓                        ↓
┌─────────────────────────────────────────────────────────────────┐
│                         Host System                              │
│                                                                   │
│         Gazebo Simulation + NS-3 Plugin                          │
│         - 로봇 물리 시뮬레이션                                      │
│         - WiFi 네트워크 시뮬레이션                                  │
│         - 로봇 제어 수신                                           │
└─────────────────────────────────────────────────────────────────┘
```

### 네트워크 분리 상세

#### WiFi Network (10.0.0.0/9)
- **목적**: 로봇 간 데이터 통신
- **경로**: nns1 ↔ NS-3 WiFi ↔ nns2
- **특징**:
  - 802.11a WiFi 시뮬레이션
  - 거리에 따른 RSSI 변화
  - 패킷 손실, 지연 발생 가능
  - 현실적인 무선 네트워크 동작
- **사용 예**: ROS2 토픽 통신 (talker/listener, 센서 데이터)

#### Direct Network (10.128.0.0/29+)
- **목적**: 네임스페이스 → 호스트 Gazebo 제어
- **경로**: nns1 (10.128.0.2/29) ↔ 호스트 (10.128.0.1/29)
- **특징**:
  - 각 네임스페이스는 별도의 /29 서브넷 사용
  - 네임스페이스 간 완전히 격리됨
  - NS-3 우회, 즉각적 반응
  - 안정적인 제어 경로
- **사용 예**: 로봇 제어 명령 (forward, backward, left, right)

## 빠른 시작

### 자동 설정 (권장)

```bash
cd /home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin

# 1. 네트워크 검증
sudo ./verify_network.sh 2

# 2. 통합 시스템 시작
sudo ./start_dual_network.sh 2
```

### 수동 설정

#### 1. 이중 네트워크 설정

```bash
cd /home/user/realgazebo/ns3_gazebo/scripts

# Direct 네트워크 포함하여 설정 (중요: --include_direct 옵션)
sudo python3 nns_setup.py setup -c 2 --include_direct

# 검증
ip netns list  # nns1, nns2 확인
ip addr show | grep "direct_br"  # direct_br1, direct_br2 확인
```

#### 2. Gazebo 시작

```bash
cd /home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/build"
gz sim ns3_gazebo_ros2.sdf
```

#### 3. 로봇 Spawn

```bash
./spawn_two_robots.sh
```

로봇 위치:
- robot1: (-3.0, 0.0, 0.5) - 왼쪽
- robot2: (3.0, 0.0, 0.5) - 오른쪽
- 초기 거리: 6.0m

**중요**: 각 로봇은 독립적인 SDF 파일을 사용합니다:
- `robot1_model.sdf`: `/model/robot1/cmd_vel` 토픽으로 제어
- `robot2_model.sdf`: `/model/robot2/cmd_vel` 토픽으로 제어

이렇게 분리하여 각 로봇을 독립적으로 제어할 수 있습니다.

#### 4. ROS2-Gazebo Relay Bridge 시작 (FastDDS Static Peers)

**새 터미널에서 (호스트):**

```bash
cd /home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin
./start_relay_bridge.sh robot1 robot2
```

이 브리지는:
- **FastDDS Static Peers**를 사용하여 네임스페이스의 ROS2 노드 발견
- 네임스페이스의 ROS2 토픽 구독 (`/robot1/cmd_vel`, `/robot2/cmd_vel`)
- Gazebo Transport로 변환 (`/model/robot1/cmd_vel`, `/model/robot2/cmd_vel`)
- Direct 네트워크(10.128.0.x)를 통해 unicast 통신

**FastDDS 동작 원리**:
- 기본 ROS2 DDS는 UDP multicast (239.255.0.1)로 노드 디스커버리
- Multicast는 네임스페이스 경계를 넘지 못함
- FastDDS Static Peers는 **미리 지정된 IP로 unicast** 전송
- Unicast는 Direct 네트워크를 통해 전달됨

**설정 파일**:
- `fastdds_host.xml`: 호스트용 (peers: nns1, nns2)
  - Direct Network 인터페이스 listening: 10.128.0.1, 10.128.0.9
- `fastdds_nns1.xml`: nns1용
  - Peers: 호스트(10.128.0.1), nns2(10.0.0.4 - WiFi)
  - Listening: Direct(10.128.0.2) + WiFi(10.0.0.1)
- `fastdds_nns2.xml`: nns2용
  - Peers: 호스트(10.128.0.9), nns1(10.0.0.1 - WiFi)
  - Listening: Direct(10.128.0.10) + WiFi(10.0.0.4)

**중요**:
- **로봇 제어 (nns → host)**: Direct Network를 통한 FastDDS Static Peers
- **로봇 간 통신 (nns1 ↔ nns2)**: WiFi Network(10.0.0.x)를 통한 ROS2 기본 multicast discovery
- nns1과 nns2는 WiFi peer로 등록되어 있어 **NS-3 시뮬레이션을 거쳐 통신**합니다

## 로봇 제어

### 네임스페이스에서 제어 (Direct Network 사용)

#### 방법 1: 편리한 스크립트 사용 (권장)

```bash
# robot1 제어 (nns1에서)
sudo ip netns exec nns1 bash -c "
    source /opt/ros/jazzy/setup.bash &&
    cd /home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin &&
    ./control_robot_from_ns.sh robot1 forward
"

# robot1 회전
sudo ip netns exec nns1 bash -c "
    source /opt/ros/jazzy/setup.bash &&
    cd /home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin &&
    ./control_robot_from_ns.sh robot1 left 0.3
"

# robot1 정지
sudo ip netns exec nns1 bash -c "
    source /opt/ros/jazzy/setup.bash &&
    cd /home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin &&
    ./control_robot_from_ns.sh robot1 stop
"

# robot2 제어 (nns2에서)
sudo ip netns exec nns2 bash -c "
    source /opt/ros/jazzy/setup.bash &&
    cd /home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin &&
    ./control_robot_from_ns.sh robot2 backward
"
```

#### 방법 2: ROS2 토픽 직접 사용

```bash
# robot1 앞으로
sudo ip netns exec nns1 bash -c "
    source /opt/ros/jazzy/setup.bash &&
    ros2 topic pub --once /robot1/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
"

# robot2 회전
sudo ip netns exec nns2 bash -c "
    source /opt/ros/jazzy/setup.bash &&
    ros2 topic pub --once /robot2/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
"
```

### 제어 흐름

```
네임스페이스 (nns1):
  ROS2 토픽 발행: /robot1/cmd_vel
       ↓ (Direct Network: 10.128.0.2 → 10.128.0.1)
호스트:
  relay_bridge.py가 ROS2 토픽 수신
       ↓ (변환)
  Gazebo Transport로 발행: /model/robot1/cmd_vel
       ↓ (Gazebo Transport)
Gazebo:
  robot1 DiffDrive 플러그인이 수신
       ↓
  로봇 움직임!
```

## 로봇 간 통신 테스트 (WiFi Network 사용)

로봇 간 ROS2 통신은 **NS-3 WiFi 네트워크를 통과**합니다. 기본 ROS2 multicast discovery가 WiFi 네트워크(10.0.0.x)를 통해 작동하며, FastDDS 설정 없이도 자동으로 통신합니다.

### Talker/Listener 예제

**터미널 1: nns1에서 talker 실행**

```bash
sudo ip netns exec nns1 bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

**터미널 2: nns2에서 listener 실행**

```bash
sudo ip netns exec nns2 bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp listener
```

**예상 결과**:
```
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
[INFO] [listener]: I heard: [Hello World: 3]
...
```

**NS-3 네트워크 효과 관찰**:
- ✅ 메시지 수신 성공 (거리 6m, RSSI -54dBm)
- 메시지 지연 발생 가능 (NS-3 시뮬레이션)
- 일부 메시지가 묶여서 도착할 수 있음
- 로봇 간 거리가 멀어지면 (>50m) 패킷 손실 발생
- Gazebo 콘솔에서 실시간 RSSI/SNR 값 확인 가능

### 네트워크 품질 확인

Gazebo 콘솔 출력:

```
=== NS-3 Multi-Robot Network Status ===
robot1 (NS-3 Node 0): (-3.00, 0.00, 0.50)
robot2 (NS-3 Node 1): (3.00, 0.00, 0.50)
Inter-robot distance: 6.00 m
  RSSI: -54.0 dBm, SNR: 40.0 dB  (Excellent signal)
Packets RX: 150
```

**거리에 따른 신호 품질**:
- **6m**: RSSI -54 dBm, SNR 40 dB → 매우 좋음, 통신 안정적
- **65m**: RSSI -82 dBm, SNR 12 dB → 매우 약함, 데이터 전송 불가능
- **권장 통신 거리**: 50m 이내

### 거리 실험

로봇을 이동시켜 통신 품질 변화를 관찰할 수 있습니다:

```bash
# robot1을 계속 전진시켜 거리 증가
sudo ip netns exec nns1 bash -c "
    source /opt/ros/jazzy/setup.bash &&
    cd /home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin &&
    ./control_robot_from_ns.sh robot1 forward
"

# Gazebo 콘솔에서 RSSI 값 관찰
# 거리가 증가할수록 RSSI 값 감소 (-54 → -60 → -70 → -82 dBm)
```

로봇 위치 리셋:
```bash
# Gazebo 서비스로 로봇 위치 초기화
gz service -s /world/ns3_gazebo_world/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req "name: 'robot1', position: {x: -3.0, y: 0.0, z: 0.5}"

gz service -s /world/ns3_gazebo_world/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req "name: 'robot2', position: {x: 3.0, y: 0.0, z: 0.5}"
```

## 네트워크 검증

### 설정 확인

```bash
# 전체 네트워크 검증
sudo ./verify_network.sh 2
```

검증 항목:
- ✓ 네임스페이스 존재
- ✓ WiFi 인터페이스 설정
- ✓ Direct 인터페이스 설정
- ✓ 라우팅 테이블
- ✓ 네임스페이스 간 격리
- ✓ 호스트 연결성

### 수동 확인

```bash
# WiFi 네트워크 확인 (nns1)
sudo ip netns exec nns1 ip addr show wifi_veth1
# 출력: inet 10.0.0.1/9

# Direct 네트워크 확인 (nns1)
sudo ip netns exec nns1 ip addr show direct_vethn1
# 출력: inet 10.128.0.2/29

# 호스트 연결 테스트
sudo ip netns exec nns1 ping -c 1 10.128.0.1
# 성공하면 Direct 네트워크 작동

# 네임스페이스 간 격리 확인 (WiFi 네트워크)
sudo ip netns exec nns1 ping -c 1 10.0.0.4
# 실패해야 정상 (NS-3 플러그인이 활성화되어야 통신 가능)
```

## NS-3 네트워크 특성

- **WiFi 표준**: 802.11a
- **데이터 레이트**: 54 Mbps (OfdmRate54Mbps)
- **네트워크 모드**: Ad-hoc (IBSS)
- **전파 모델**: YansWifiChannel (기본 설정)
- **IP 주소 범위**: 10.0.0.0/9 (WiFi), 10.128.0.0/29+ (Direct)

## 문제 해결

### ROS2 토픽이 Gazebo로 전달되지 않음 ("Waiting for at least 1 matching subscription(s)...")

이것은 **ROS2 DDS discovery 실패** 문제입니다. nns1의 ROS2 노드가 호스트의 relay_bridge를 발견하지 못하고 있습니다.

#### 원인
- ROS2 기본 DDS는 UDP multicast로 노드 디스커버리
- 네임스페이스는 multicast 트래픽을 격리
- 따라서 서로를 발견하지 못함

#### 해결책: FastDDS Static Peers 사용

1. **relay_bridge가 FastDDS로 실행 중인지 확인**
   ```bash
   ps aux | grep relay_bridge
   # start_relay_bridge.sh로 실행되었는지 확인

   tail -f /tmp/relay_bridge.log
   # "Static Peers" 관련 메시지 확인
   ```

2. **FastDDS 환경 변수 확인**
   ```bash
   # relay_bridge 프로세스의 환경 변수 확인
   ps aux | grep relay_bridge | awk '{print $2}' | xargs -I {} cat /proc/{}/environ | tr '\0' '\n' | grep FASTRTPS

   # 출력되어야 할 내용:
   # FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds_host.xml
   # RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ```

3. **네임스페이스에서 FastDDS 설정 확인**
   ```bash
   sudo ip netns exec nns1 bash -c "
       source /opt/ros/jazzy/setup.bash &&
       cd /home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin &&
       ./control_robot_from_ns.sh robot1 forward
   "

   # 출력에서 확인:
   # [nns1] Using FastDDS config: fastdds_nns1.xml
   ```

4. **ROS2 노드 디스커버리 테스트**
   ```bash
   # 호스트에서 relay_bridge 실행 후

   # nns1에서 노드 목록 확인
   sudo ip netns exec nns1 bash -c "
       export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin/fastdds_nns1.xml
       export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
       source /opt/ros/jazzy/setup.bash
       ros2 node list
   "

   # 출력: /ros2_gazebo_relay_bridge 보여야 함!
   ```

5. **Direct 네트워크 설정 확인**
   ```bash
   sudo ./verify_network.sh 2

   # Direct 네트워크 인터페이스 확인
   sudo ip netns exec nns1 ip addr show direct_vethn1
   # inet 10.128.0.2/29 ...
   ```

6. **호스트 연결 테스트**
   ```bash
   sudo ip netns exec nns1 ping -c 2 10.128.0.1  # nns1 → 호스트
   # 성공해야 함 (2 packets received)
   ```

#### 다시 시작하기

문제가 계속되면 완전히 다시 시작:

```bash
# 1. relay_bridge 중지
pkill -f relay_bridge

# 2. relay_bridge 다시 시작 (FastDDS 설정 포함)
cd /home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin
./start_relay_bridge.sh robot1 robot2

# 3. 로봇 제어 시도
sudo ip netns exec nns1 bash -c "
    source /opt/ros/jazzy/setup.bash &&
    cd /home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin &&
    ./control_robot_from_ns.sh robot1 forward
"
```

### 로봇 간 통신이 안 됨 (WiFi)

#### 1. 기본 연결 확인

```bash
# nns1에서 talker 실행
sudo ip netns exec nns1 bash -c "
    source /opt/ros/jazzy/setup.bash &&
    ros2 run demo_nodes_cpp talker
"

# nns2에서 listener 실행
sudo ip netns exec nns2 bash -c "
    source /opt/ros/jazzy/setup.bash &&
    ros2 run demo_nodes_cpp listener
"
```

메시지가 수신되지 않으면:

#### 2. NS-3 플러그인 로딩 확인
- Gazebo 콘솔에서 "NS3GazeboWorld configured!" 메시지 확인

#### 3. TAP 장치 확인
```bash
ip addr show | grep wifi_tap
# wifi_tap1, wifi_tap2가 있어야 함
```

#### 4. 거리 확인
로봇 간 거리가 너무 멀면 신호 약화로 통신 불가:

```bash
# 로봇 위치를 원위치로 리셋
gz service -s /world/ns3_gazebo_world/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req "name: 'robot1', position: {x: -3.0, y: 0.0, z: 0.5}"

gz service -s /world/ns3_gazebo_world/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req "name: 'robot2', position: {x: 3.0, y: 0.0, z: 0.5}"
```

Gazebo 콘솔에서 확인:
- RSSI > -70 dBm: 통신 가능
- RSSI < -80 dBm: 통신 불가능 (거리 너무 멀음)

#### 5. WiFi 네트워크 ping 테스트
```bash
# nns2에서 nns1로 ping
sudo ip netns exec nns2 ping -c 3 10.0.0.1

# 성공 예시:
# 64 bytes from 10.0.0.1: icmp_seq=1 ttl=64 time=0.477 ms
# RTT가 0.5ms~100ms로 다양 → NS-3 시뮬레이션 작동 중
```

### 네임스페이스 설정 실패

```bash
# 기존 설정 제거
sudo python3 ../scripts/nns_setup.py teardown -c 2 --include_direct

# 재설정
sudo python3 ../scripts/nns_setup.py setup -c 2 --include_direct

# 검증
sudo ./verify_network.sh 2
```

## 시스템 종료

```bash
# 1. relay_bridge 중지
pkill -f relay_bridge.py

# 2. Gazebo 중지
pkill -9 gz

# 3. 네트워크 네임스페이스 제거
cd /home/user/realgazebo/ns3_gazebo/scripts
sudo python3 nns_setup.py teardown -c 2 --include_direct
```

## 고급 사용법

### 더 많은 로봇 추가

```bash
# 3개 로봇 시뮬레이션
sudo python3 nns_setup.py setup -c 3 --include_direct

# relay_bridge에 robot3 추가
python3 relay_bridge.py robot1 robot2 robot3
```

### 커스텀 속도 제어

```bash
# 사용자 정의 선속도 및 각속도
sudo ip netns exec nns1 bash -c "
    source /opt/ros/jazzy/setup.bash &&
    cd /home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin &&
    ./control_robot_from_ns.sh robot1 custom 0.7 0.3
"
```

### 네트워크 품질 실험

로봇을 이동시켜 거리에 따른 네트워크 품질 변화 확인:

**실험 절차**:

1. **초기 상태 확인** (거리 6m)
   ```bash
   # nns1에서 talker 시작
   sudo ip netns exec nns1 bash -c "
       source /opt/ros/jazzy/setup.bash &&
       ros2 run demo_nodes_cpp talker
   "

   # nns2에서 listener 시작
   sudo ip netns exec nns2 bash -c "
       source /opt/ros/jazzy/setup.bash &&
       ros2 run demo_nodes_cpp listener
   "
   # 메시지 정상 수신, RSSI: -54 dBm
   ```

2. **robot1을 앞으로 이동** (거리 증가)
   ```bash
   sudo ip netns exec nns1 bash -c "
       source /opt/ros/jazzy/setup.bash &&
       cd /home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin &&
       ./control_robot_from_ns.sh robot1 forward
   "
   ```

3. **Gazebo 콘솔에서 RSSI/SNR 값 관찰**
   - 10~20m: RSSI -60~-65 dBm → 안정적
   - 30~40m: RSSI -70~-75 dBm → 불안정, 지연 증가
   - 50m+: RSSI -80 dBm 이하 → 패킷 손실 시작
   - 65m+: RSSI -82 dBm 이하 → 통신 불가능

4. **listener에서 메시지 수신 패턴 관찰**
   - 거리 증가 → 메시지 지연 증가
   - RSSI < -80 dBm → 메시지 수신 중단

5. **로봇 위치 리셋 후 재테스트**
   ```bash
   gz service -s /world/ns3_gazebo_world/set_pose \
     --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
     --req "name: 'robot1', position: {x: -3.0, y: 0.0, z: 0.5}"

   # 다시 메시지 정상 수신 확인
   ```

## 참고 자료

- [CLAUDE.md](../../CLAUDE.md) - 전체 시스템 아키텍처
- [README.md](README.md) - NS-3 Gazebo 플러그인 설명
- [nns_setup.py](../scripts/nns_setup.py) - 네트워크 네임스페이스 설정 스크립트
