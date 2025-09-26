# NS3-Gazebo 시스템 업그레이드 계획서

## 프로젝트 개요

### 업그레이드 목표
- **NS-3**: 버전 3.29 → 3.45
- **Gazebo**: Gazebo 9 → Gazebo Harmonic (gz_sim8)
- **ROS2**: Jazzy 환경 최적화

### 현재 상태
- NS-3 3.45는 이미 설치되어 있음 (`ns-allinone-3.45/` 디렉토리)
- 기존 코드는 NS-3 3.29 API 및 Gazebo 9 기반
- ROS2 Jazzy 환경에서 동작

---

## ✅ Phase 1: 준비 및 백업 (완료)

### 1.1 백업 생성 ✅
```bash
# 전체 프로젝트 백업
cp -r /home/user/realgazebo/ns3_gazebo /home/user/realgazebo/ns3_gazebo_backup

# Git 커밋 (변경사항 추적용) - 사용자가 직접 처리
cd /home/user/realgazebo/ns3_gazebo
git add .
git commit -m "Pre-upgrade backup: NS-3 3.29 + Gazebo 9 baseline"
```
**결과**: 백업 생성 완료 (160M → 152M, 파일 수 동일)

### 1.2 의존성 확인 ✅
```bash
# Gazebo Harmonic 설치 확인 - Gazebo Sim 8.9.0 확인
gz sim --versions

# ROS2 Jazzy 환경 확인 - 정상 동작, 모든 패키지 최신
ros2 doctor --report

# NS-3 3.45 빌드 상태 확인 - 모든 필요 모듈 활성화
cd ns-allinone-3.45/ns-3.45
./ns3 configure
```
**결과**: 모든 의존성 정상 설치 및 동작 확인

### 1.3 환경 설정 ✅
- [x] Docker 환경 준비 (현재 Docker 컨테이너에서 동작중)
- [x] 빌드 도구 업데이트 확인 (CMake 3.28.3, Colcon 최신)
- [x] 테스트 환경 구성 (GCC 13.3.0, CCache 활성화)

**완료 기준**: ✅ 백업 완료, ✅ 의존성 설치 확인, ✅ 테스트 환경 준비
**소요 시간**: 10분 (계획: 1일)

---

## 🔧 Phase 2: NS-3 업그레이드 (3.29 → 3.45)

### 2.1 빌드 시스템 수정

#### 2.1.1 CMakeLists.txt 경로 업데이트
**수정 대상 파일**:
1. `ns3_gazebo_plugin/CMakeLists.txt`
2. `ns3_gazebo_ws/src/diff_drive_ns3/CMakeLists.txt`
3. `ns3_testbed/ns3_mobility/CMakeLists.txt`
4. `ns3_testbed_simtime/ns3_simtime_support/CMakeLists.txt`
5. `ns3_wifi_tap_test/CMakeLists.txt`

**변경사항**:
```cmake
# 기존
include_directories(src ~/repos/ns-3-allinone/ns-3.29/build)
link_directories(~/repos/ns-3-allinone/ns-3.29/build/lib)

# 수정후
include_directories(src ${CMAKE_CURRENT_SOURCE_DIR}/../ns-allinone-3.45/ns-3.45/build)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../ns-allinone-3.45/ns-3.45/build/lib)
```

#### 2.1.2 라이브러리 버전 업데이트
```cmake
# 기존
target_link_libraries(target_name
  ns3.29-core-debug
  ns3.29-network-debug
  ns3.29-internet-debug
  ns3.29-wifi-debug
  ns3.29-mobility-debug
  ns3.29-tap-bridge-debug
)

# 수정후
target_link_libraries(target_name
  ns3.45-core-debug
  ns3.45-network-debug
  ns3.45-internet-debug
  ns3.45-wifi-debug
  ns3.45-mobility-debug
  ns3.45-tap-bridge-debug
)
```

### 2.2 소스 코드 API 업데이트

#### 2.2.1 WiFi Standard API 변경
**수정 대상 파일**:
- `ns3_gazebo_plugin/ns3_gazebo_world.cpp`
- `ns3_gazebo_ws/src/diff_drive_ns3/src/diff_drive_ns3_ros2.cpp`
- `ns3_wifi_tap_test/ns3_wifi_tap_test.cpp`

**변경사항**:
```cpp
// 기존
wifi.SetStandard(ns3::WIFI_PHY_STANDARD_80211a);

// 수정후
wifi.SetStandard(ns3::WIFI_STANDARD_80211a);
```

#### 2.2.2 Helper 클래스 생성자 변경
```cpp
// 기존
ns3::YansWifiChannelHelper wifiChannel(ns3::YansWifiChannelHelper::Default());
ns3::YansWifiPhyHelper wifiPhy(ns3::YansWifiPhyHelper::Default());

// 수정후
ns3::YansWifiChannelHelper wifiChannel;  // 기본 생성자 사용
ns3::YansWifiPhyHelper wifiPhy;
```

### 2.3 빌드 테스트
```bash
# 각 모듈별 개별 빌드 테스트
cd ns3_gazebo_plugin && mkdir -p build && cd build
cmake .. && make

cd ../../ns3_gazebo_ws && colcon build --packages-select diff_drive_ns3_ros2

cd ../ns3_testbed/ns3_mobility && mkdir -p build && cd build
cmake .. && make
```

**완료 기준**: 모든 NS-3 관련 모듈이 오류 없이 빌드 완료

---

## 🎯 Phase 3: Gazebo Harmonic 마이그레이션

### 3.1 헤더 파일 및 네임스페이스 수정

#### 3.1.1 Include 구문 업데이트
**수정 대상 파일**:
- `ns3_gazebo_plugin/ns3_gazebo_world.cpp`
- `ns3_gazebo_plugin/hello_world.cpp`

**변경사항**:
```cpp
// 기존
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>

// 수정후 (Gazebo Harmonic)
#include <gz/sim/Server.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components.hh>
#include <gz/math/Pose3.hh>
```

#### 3.1.2 네임스페이스 및 클래스 업데이트
```cpp
// 기존
ignition::math::Pose3d pose = model_ptr->WorldPose();
gazebo::WorldPlugin

// 수정후
gz::math::Pose3d pose = model.worldPose();
gz::sim::System
```

### 3.2 CMake 빌드 시스템 업데이트
```cmake
# 기존
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
target_link_libraries(target ${GAZEBO_LIBRARIES})

# 수정후
find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-math7 REQUIRED)

target_link_libraries(target
    gz-sim8::gz-sim8
    gz-math7::gz-math7
)
```

### 3.3 World 파일 업데이트
**수정 대상**: `ns3_gazebo_plugin/gazebo_ros_diff_drive_ns3_gazebo.world`

```xml
<!-- 기존 -->
<sdf version="1.6">

<!-- 수정후 -->
<sdf version="1.8">
  <!-- Gazebo Harmonic 호환 업데이트 -->
```

**완료 기준**: Gazebo Harmonic에서 플러그인 로드 및 기본 시뮬레이션 실행 가능

---

## 🔍 Phase 4: 통합 테스트 및 검증

### 4.1 개별 모듈 테스트

#### 4.1.1 NS-3 단독 테스트
```bash
cd ns3_wifi_tap_test
./ns3_wifi_tap_test
```

#### 4.1.2 Gazebo 플러그인 테스트
```bash
gz sim gazebo_ros_diff_drive_ns3_gazebo.world
```

#### 4.1.3 ROS2 통신 테스트
```bash
# Terminal 1: ROS2 노드 실행
cd ns3_gazebo_ws
colcon build && source install/setup.bash
ros2 run diff_drive_ns3_ros2 diff_drive_ns3_ros2

# Terminal 2: 토픽 확인
ros2 topic list
ros2 topic echo /demo/odom_demo
```

### 4.2 통합 시스템 테스트

#### 4.2.1 전체 파이프라인 테스트
```bash
# 1. 네트워크 네임스페이스 설정
cd scripts
sudo python3 nns_setup.py --setup-nns 1 --setup-wifi 1

# 2. NS-3 + Gazebo + ROS2 통합 실행
cd ns3_testbed
python3 testbed_runner.py
```

#### 4.2.2 성능 및 안정성 테스트
- [ ] 장시간 실행 안정성 (30분+)
- [ ] 메모리 누수 검사
- [ ] CPU 사용량 모니터링
- [ ] 네트워크 지연시간 측정

### 4.3 기능 회귀 테스트
- [ ] WiFi 통신 품질 확인
- [ ] 로봇 제어 응답성 테스트
- [ ] 시뮬레이션 동기화 검증
- [ ] ROS2 메시지 전달 정확성

**완료 기준**: 모든 기존 기능이 정상 동작, 성능 저하 없음

---

## 📚 Phase 5: 문서화 및 정리

### 5.1 코드 문서화
- [ ] API 변경사항 주석 추가
- [ ] 새로운 의존성 README 업데이트
- [ ] 예제 코드 업데이트

### 5.2 사용자 가이드 업데이트
```markdown
# 새로운 빌드 절차
1. Gazebo Harmonic 설치
2. NS-3 3.45 빌드
3. ROS2 Jazzy 환경 설정
4. ns3_gazebo 프로젝트 빌드
```

### 5.3 버전 관리
```bash
# 업그레이드 완료 태그
git add .
git commit -m "Upgrade complete: NS-3 3.45 + Gazebo Harmonic"
git tag -a v2.0.0 -m "NS-3 3.45 + Gazebo Harmonic + ROS2 Jazzy"
```

**완료 기준**: 완전한 문서화, 새로운 사용자도 쉽게 설치/사용 가능

---

## ⚠️ 위험도 분석 및 대응책

### 높은 위험도
| 위험 요소 | 발생 확률 | 영향도 | 대응책 |
|-----------|-----------|--------|--------|
| NS-3 API 호환성 문제 | 중간 | 높음 | 단계별 테스트, API 문서 참조 |
| Gazebo 플러그인 동작 불가 | 중간 | 높음 | 기존 버전과 병렬 유지 |
| ROS2 메시지 타입 변경 | 낮음 | 중간 | 메시지 호환성 확인 |

### 중간 위험도
| 위험 요소 | 발생 확률 | 영향도 | 대응책 |
|-----------|-----------|--------|--------|
| 성능 저하 | 중간 | 중간 | 벤치마크 비교, 최적화 |
| 메모리 사용량 증가 | 낮음 | 중간 | 프로파일링, 모니터링 |

### 복구 계획
```bash
# 긴급 복구 절차
1. 백업에서 복원: cp -r ns3_gazebo_backup/* ns3_gazebo/
2. Git 롤백: git reset --hard [백업_커밋_해시]
3. 개별 모듈 선택적 복구
```

---

## 📅 타임라인 및 마일스톤

### 전체 일정: **2-3주**

| 단계 | 예상 소요시간 | 마일스톤 |
|------|---------------|----------|
| Phase 1: 준비 및 백업 | 1일 | ✅ 백업 완료, 환경 준비 |
| Phase 2: NS-3 업그레이드 | 3-5일 | ✅ NS-3 3.45 빌드 성공 |
| Phase 3: Gazebo 마이그레이션 | 4-6일 | ✅ Gazebo Harmonic 통합 |
| Phase 4: 통합 테스트 | 3-4일 | ✅ 전체 시스템 검증 완료 |
| Phase 5: 문서화 | 2-3일 | ✅ 사용자 가이드 완성 |

### 체크포인트
- **1주차 말**: NS-3 업그레이드 완료
- **2주차 말**: Gazebo 마이그레이션 완료
- **3주차 말**: 전체 시스템 검증 및 문서화 완료

---

## 🎯 성공 기준

### 기능적 요구사항
- [ ] 모든 기존 시뮬레이션 시나리오가 정상 동작
- [ ] WiFi 네트워크 시뮬레이션 품질 유지
- [ ] ROS2 메시지 통신 정상
- [ ] 실시간 시뮬레이션 동기화 유지

### 비기능적 요구사항
- [ ] 빌드 시간 30% 이내 증가
- [ ] 런타임 성능 10% 이내 변화
- [ ] 메모리 사용량 20% 이내 증가
- [ ] 문서화 완료 (100%)

### 품질 지표
- [ ] 단위 테스트 통과율 100%
- [ ] 통합 테스트 통과율 100%
- [ ] 코드 커버리지 80% 이상 유지
- [ ] 사용자 피드백 긍정적

---

## 📞 연락처 및 지원

**기술 지원**:
- NS-3 커뮤니티: https://groups.google.com/g/ns-3-users
- Gazebo 포럼: https://community.gazebosim.org/
- ROS2 문서: https://docs.ros.org/en/jazzy/

**내부 연락처**:
- 프로젝트 리더: [담당자 이름]
- 시스템 관리자: [담당자 이름]
- 테스트 팀: [담당자 이름]

---

*이 문서는 업그레이드 진행 상황에 따라 지속적으로 업데이트됩니다.*