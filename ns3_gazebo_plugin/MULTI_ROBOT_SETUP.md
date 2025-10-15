# Multi-Robot NS-3 Network Simulation Setup

이 가이드는 2개의 로봇을 각각의 네트워크 네임스페이스(nns)에서 실행하고, NS-3를 통해 WiFi 네트워크로 통신하는 방법을 설명합니다.

## 시스템 구성

- **Gazebo World**: 빈 world (`ns3_gazebo_ros2.sdf`)
- **NS-3 Network**: 2개의 WiFi 노드 (802.11a, Ad-hoc mode)
- **Network Namespaces**: nns1, nns2
- **Robots**: robot1 (nns1), robot2 (nns2)

## 실행 단계

### 1. 네트워크 네임스페이스 확인

이미 nns1, nns2가 생성되어 있습니다:

```bash
ip netns list
# 출력: nns1, nns2
```

없다면 생성

```bash
cd /home/user/realgazebo/ns3_gazebo/scripts
sudo python3 nns_setup.py setup -c 2
```

### 2. Gazebo 실행

빈 world와 NS-3 플러그인을 실행합니다:

```bash
cd /home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/build"
gz sim ns3_gazebo_ros2.sdf
```

**주의**: Gazebo가 완전히 로드될 때까지 기다립니다 (약 5-10초).

### 3. 로봇 Spawn (새 터미널)

```bash
cd /home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin
./spawn_two_robots.sh
```

이 스크립트는:
- robot1을 nns1 네임스페이스에서 위치 (-3.0, 0.0, 0.5)에 spawn
- robot2를 nns2 네임스페이스에서 위치 (3.0, 0.0, 0.5)에 spawn
- 두 로봇 간 거리: 6.0m

### 4. NS-3 네트워크 상태 확인

Gazebo 콘솔 출력에서 다음과 같은 정보를 확인할 수 있습니다:

```
=== NS-3 Multi-Robot Network Status ===
robot1 (NS-3 Node 0): (-3.00, 0.00, 0.50)
  RSSI: -XX.X dBm, SNR: XX.X dB
  Packets RX: XXXX

robot2 (NS-3 Node 1): (3.00, 0.00, 0.50)
  RSSI: -XX.X dBm, SNR: XX.X dB
  Packets RX: XXXX

Inter-robot distance: 6.00 m
======================================
```

### 5. ROS2 Topic 통신 테스트


터미널 1 (robot1에서 publish):
```bash
sudo ip netns exec nns1 bash -c "
    source /opt/ros/jazzy/setup.bash
    ros2 topic pub /chatter std_msgs/msg/String 'data: \"Hello from robot1\"' --rate 1
"
```

터미널 2 (robot2에서 subscribe):
```bash
sudo ip netns exec nns2 bash -c "
    source /opt/ros/jazzy/setup.bash
    ros2 topic echo /chatter
"
```

### 6. Topic 리스트 확인

각 네임스페이스에서 보이는 topic을 확인:

```bash
# nns1에서
sudo ip netns exec nns1 bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list"

# nns2에서
sudo ip netns exec nns2 bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list"
```

## 로봇 제어

각 로봇은 differential drive를 사용합니다. Gazebo의 cmd_vel topic으로 제어할 수 있습니다:

```bash
# robot1 제어 (nns1에서)
sudo ip netns exec nns1 bash -c "
    source /opt/ros/jazzy/setup.bash
    ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}' --once
"

# robot2 제어 (nns2에서)
sudo ip netns exec nns2 bash -c "
    source /opt/ros/jazzy/setup.bash
    ros2 topic pub /robot2/cmd_vel geometry_msgs/msg/Twist '{linear: {x: -0.5}, angular: {z: 0.0}}' --once
"
```

**참고**: Gazebo의 topic remapping이 필요할 수 있습니다. 기본적으로 각 로봇의 cmd_vel topic은 `/model/robot1/cmd_vel` 형태입니다.

## NS-3 네트워크 특성

- **WiFi 표준**: 802.11a
- **데이터 레이트**: 54 Mbps (OfdmRate54Mbps)
- **네트워크 모드**: Ad-hoc (IBSS)
- **전파 모델**: YansWifiChannel (기본 설정)
- **IP 주소 범위**: 10.1.1.0/24

### RSSI 기준

- RSSI > -50 dBm: Excellent (최대 속도)
- RSSI > -60 dBm: Good (54 Mbps)
- RSSI > -70 dBm: Fair (속도 감소)
- RSSI > -80 dBm: Weak (불안정)
- RSSI ≤ -80 dBm: Very Weak (패킷 손실)

## 문제 해결

### 로봇이 Gazebo에 나타나지 않음

1. Gazebo가 완전히 로드되었는지 확인
2. SDF 파일 경로 확인: `ls -la robot_model.sdf`
3. ROS2 설치 확인: `ros2 pkg list | grep ros_gz_sim`

### ROS2 topic이 보이지 않음

1. 네임스페이스가 활성화되어 있는지 확인: `ip netns list`
2. TAP bridge 설정 확인 (NS-3 플러그인에서 자동 설정)
3. ROS2 domain ID가 일치하는지 확인

### NS-3 네트워크 상태가 업데이트되지 않음

1. Gazebo 콘솔에서 "NS3GazeboWorld Plugin Configure" 메시지 확인
2. "Matched robot model" 메시지 확인
3. 로봇 이름이 "robot"으로 시작하는지 확인

## 추가 정보

- NS-3 로그 파일: `ns3_gazebo_ros2_log.csv`
- 플러그인 소스: `ns3_gazebo_world.cpp`
- World 파일: `ns3_gazebo_ros2.sdf`
- 로봇 모델: `robot_model.sdf`

## 정리

시뮬레이션 종료 후:

```bash
# Gazebo 종료 (Ctrl+C)

# 네임스페이스 정리 (선택사항)
cd /home/user/realgazebo/ns3_gazebo/scripts
sudo python3 nns_setup.py teardown -c 2
```
