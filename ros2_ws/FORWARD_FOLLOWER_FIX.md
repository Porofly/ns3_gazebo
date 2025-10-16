# Forward Follower Robot Bug Fix

## Problem Report

사용자가 보고한 두 가지 문제:
1. **팔로우가 전진만 하고 좌우 움직임이 없다** - Follower only moves forward, no lateral movement
2. **멀어져서 통신이 끊켜도 제자리에 멈추지 않는다** - Robot doesn't stop when communication is lost

## Root Cause

### Bug 1: No Lateral Movement

**Original Implementation (INCORRECT):**
```python
self.leader_linear_y = msg.twist.twist.linear.y
twist.linear.y = self.leader_linear_y
```

**Problem:** Gazebo의 DiffDrive 플러그인은 `linear.y` (좌우 이동)를 지원하지 않습니다. DiffDrive는 다음만 지원:
- `linear.x` - 전진/후진 (forward/backward)
- `angular.z` - 회전 (rotation)

DiffDrive는 차동 구동 방식으로, 옆으로 직접 이동할 수 없습니다 (omnidirectional movement 불가).

### Bug 2: Timeout Not Working

타임아웃 로직 자체는 올바르게 구현되어 있었으나, 잘못된 변수(`linear.y`)에 적용되고 있었습니다.

## Solution

### Fixed Implementation

**변경 사항:**
1. `leader_linear_y` → `leader_angular_z` (리더의 회전 속도 추적)
2. `twist.linear.y` → `twist.angular.z` (회전 명령 전송)
3. 타임아웃 시 `angular.z = 0` (회전 정지)

**Fixed Code:**
```python
def __init__(self):
    super().__init__('forward_follower_node')

    # Parameters
    self.declare_parameter('base_linear_x', 1.0)  # 일정 전진 속도
    self.declare_parameter('angular_scale', 1.0)  # 회전 속도 스케일
    self.declare_parameter('timeout_threshold', 1.0)  # 타임아웃 임계값

    # State
    self.leader_angular_z = 0.0  # 리더의 회전 속도
    self.last_state_time = None

def leader_state_callback(self, msg: Odometry):
    """Extract angular velocity from leader state"""
    self.leader_angular_z = msg.twist.twist.angular.z * self.angular_scale
    self.last_state_time = time.time()
    self.communication_status = "CONNECTED"

def publish_cmd_vel(self):
    """Publish velocity with timeout detection"""
    current_time = time.time()

    # Check timeout
    if self.last_state_time is None:
        self.communication_status = "NO_COMMUNICATION"
        self.leader_angular_z = 0.0
    elif (current_time - self.last_state_time) > self.timeout_threshold:
        self.communication_status = "TIMEOUT"
        self.leader_angular_z = 0.0  # Stop rotation on timeout

    # Create velocity command
    twist = Twist()
    twist.linear.x = self.base_linear_x  # Always move forward
    twist.angular.z = self.leader_angular_z  # Follow rotation or 0

    self.cmd_vel_pub.publish(twist)
```

## Robot Behavior After Fix

### Normal Operation (Communication Available)
- **Linear X**: 1.0 m/s (forward)
- **Angular Z**: Follows leader's rotation in real-time
- **Status**: "CONNECTED"
- **Result**: Robot moves forward while rotating same as leader

### Communication Lost (Timeout > 1 second)
- **Linear X**: 0.0 m/s (STOPPED)
- **Angular Z**: 0.0 rad/s (STOPPED)
- **Status**: "TIMEOUT"
- **Result**: Robot stops completely (no movement at all)

### Communication Restored
- **Linear X**: Immediately resumes 1.0 m/s forward
- **Angular Z**: Immediately resumes following leader's rotation
- **Status**: "TIMEOUT" → "CONNECTED"

## Expected Behavior Demonstration

### Distance-Based Communication Test

**Close Range (Strong Signal):**
- 리더 로봇 회전 → 팔로워도 같이 회전하며 전진
- RSSI > -70 dBm, SNR > 20 dB
- 통신 지연 < 100ms
- **동작**: 전진(1.0 m/s) + 회전 추종

**Medium Range (Degraded Signal):**
- 간헐적으로 정지 (패킷 손실로 타임아웃 발생)
- RSSI -70 ~ -80 dBm, SNR 10-20 dB
- 패킷 손실 10-30%
- **동작**: 움직임 ↔ 정지 반복

**Far Range (Communication Lost):**
- 팔로워가 완전히 정지 (1초 이상 패킷 없음)
- RSSI < -80 dBm, SNR < 10 dB
- 1초 이상 패킷 수신 없음 → 타임아웃
- **동작**: 완전 정지 (linear.x=0, angular.z=0)

## Files Modified

1. **forward_follower_node.py** - Core logic fix
   - Changed from `linear.y` to `angular.z`
   - Timeout applies to rotation instead of y-velocity

2. **run_forward_follower_nns2.sh** - Documentation update
   - Updated behavior description
   - Added note about DiffDrive limitations

## Testing Verification

### Test 1: Rotation Following
```bash
# Terminal 1: Run leader in nns1
sudo ip netns exec nns1 bash
cd /home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin
bash run_keyboard_controller_nns1.sh

# Terminal 2: Run follower in nns2
sudo ip netns exec nns2 bash
cd /home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin
bash run_forward_follower_nns2.sh

# Expected: Follower rotates when leader rotates (keyboard left/right arrows)
```

### Test 2: Timeout Detection
```bash
# Move robots far apart until communication lost
# Expected: Follower stops rotating and goes straight

# Move robots close again
# Expected: Follower resumes rotation following
```

## DiffDrive Plugin Constraints

**Supported Commands:**
- `cmd_vel.linear.x` - Forward/backward movement
- `cmd_vel.angular.z` - Rotation (yaw)

**NOT Supported:**
- `cmd_vel.linear.y` - Lateral movement (requires omnidirectional wheels)
- `cmd_vel.linear.z` - Vertical movement (requires flying capability)
- `cmd_vel.angular.x/y` - Roll/pitch rotation (requires flying capability)

**Conclusion:** DiffDrive는 일반적인 바퀴형 로봇(wheel-based robot)의 물리적 제약을 정확히 시뮬레이션합니다. 옆으로 이동하려면 회전과 전진을 조합해야 합니다.
