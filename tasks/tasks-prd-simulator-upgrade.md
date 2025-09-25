## Relevant Files

- **Build System & Configuration**
  - `ns3_gazebo_plugin/CMakeLists.txt` - Gazebo 플러그인 빌드 스크립트. Gazebo 및 ns-3 라이브러리 링크 수정 필요.
  - `ns3_gazebo_ws/src/diff_drive_ns3/CMakeLists.txt` - ROS2 노드 빌드 스크립트. ns-3 라이브러리 링크 수정 필요.
  - `ns3_testbed/cpp_testbed_runner/CMakeLists.txt` - C++ 테스트베드 빌드 스크립트. ns-3 라이브러리 링크 수정 필요.
  - `ns3_wifi_tap_test/CMakeLists.txt` - TAP 브리지 테스트 유틸리티 빌드 스크립트. ns-3 라이브러리 링크 수정 필요.
  - `ns3_testbed_simtime/cpp_testbed_runner/CMakeLists.txt` - Sim-time 테스트베드 빌드 스크립트. ns-3 및 `shared_simtime` 라이브러리 링크 수정 필요.
  - `ns3_testbed_simtime/ns3_simtime_support/CMakeLists.txt` - `shared_simtime` 라이브러리 빌드 스크립트.
  - `*/package.xml` - 각 ROS2 패키지의 의존성 정의 파일. Gazebo 관련 의존성 수정 필요.

- **C++ Source Code (API Migration)**
  - `src/nns_ns3_wifi.cc` - ns-3 API 마이그레이션 필요.
  - `ns3_gazebo_plugin/ns3_gazebo_world.cpp` - ns-3 API 및 Gazebo 플러그인 API 동시 마이그레이션 필요.
  - `ns3_gazebo_ws/src/diff_drive_ns3/` - `diff_drive_ns3_ros2.cpp`, `diff_drive_robot.cpp` 등 ns-3 API 마이그레이션 필요.
  - `ns3_testbed/cpp_testbed_runner/src/` - `testbed_robot_callbacks.cpp` 등 ns-3 API 마이그레이션 필요.
  - `ns3_testbed_simtime/` - `ns3_simtime_support/`의 ns-3 소스 코드 패치 재적용 및 `cpp_testbed_runner`의 시간 측정 로직 검증 필요.

- **Gazebo World Files**
  - `ns3_gazebo_plugin/gazebo_ros_diff_drive_ns3_gazebo.world` - SDF 버전 및 플러그인 선언 방식 등 Gazebo Harmonic 형식으로의 마이그레이션 필요.

### Notes

- 이 작업 목록은 `/tasks/prd-simulator-upgrade.md` 문서에 기반합니다.
- 업그레이드 작업은 상호 의존성이 높으므로, 한 단계의 완료가 다음 단계의 전제 조건이 됩니다.
- API 변경점 조사는 실제 코드 수정에 앞서 선행되어야 합니다.

## Tasks

- [ ] **1.0 사전 조사 및 개발 환경 설정**
  - [ ] 1.1 ns-3.45와 Gazebo Harmonic(Ignition Gazebo)을 개발 환경에 설치하고 기본 예제가 동작하는지 확인합니다.
  - [ ] 1.2 ns-3.29에서 3.45까지의 릴리스 노트를 검토하여 이 프로젝트와 관련된 주요 API 변경점(특히 `TapBridge`, `WifiHelper`, `RealtimeSimulatorImpl`) 목록을 작성합니다.
  - [ ] 1.3 Gazebo 9에서 Gazebo Harmonic으로의 마이그레이션 가이드를 검토하여 플러그인 API(`WorldPlugin` -> `System`), SDF 버전, ROS-Gazebo 브리지 관련 변경점 목록을 작성합니다.

- [ ] **2.0 빌드 시스템 (CMake) 업그레이드**
  - [ ] 2.1 `Relevant Files`에 명시된 모든 `CMakeLists.txt` 파일에서 `find_package` 명령을 수정하여 새로운 버전의 ns-3 및 Gazebo 라이브러리를 찾도록 변경합니다.
  - [ ] 2.2 각 `CMakeLists.txt`의 `include_directories` 및 `target_link_libraries` 경로와 라이브러리 이름을 새 버전에 맞게 수정합니다.
  - [ ] 2.3 각 `package.xml` 파일의 `<build_depend>` 및 `<exec_depend>` 태그를 검토하고, Gazebo Harmonic 관련 의존성으로 변경합니다.

- [ ] **3.0 ns-3 API 마이그레이션**
  - [ ] 3.1 가장 간단한 프로젝트인 `ns3_wifi_tap_test`의 C++ 코드를 먼저 수정하여 ns-3.45 API로 컴파일 및 실행되는 것을 확인합니다.
  - [ ] 3.2 `src/nns_ns3_wifi.cc`의 ns-3 API를 수정합니다.
  - [ ] 3.3 `ns3_gazebo_plugin`, `ns3_gazebo_ws`, `ns3_testbed` 내 C++ 코드의 ns-3 관련 API를 모두 수정합니다.

- [ ] **4.0 Gazebo API 마이그레이션**
  - [ ] 4.1 `ns3_gazebo_plugin/ns3_gazebo_world.cpp`의 `gazebo::WorldPlugin`을 Gazebo Harmonic의 `gz::sim::System` 플러그인 인터페이스에 맞게 재작성합니다.
  - [ ] 4.2 `gazebo_ros_diff_drive_ns3_gazebo.world` 파일을 Gazebo Harmonic이 사용하는 최신 SDF 형식으로 변환하고, 플러그인 선언 방식을 수정합니다.
  - [ ] 4.3 Gazebo 9의 `diff_drive` 플러그인을 대체하는 Gazebo Harmonic의 플러그인(예: `gz-transport-ros2-control`)을 조사하고 적용합니다.

- [ ] **5.0 `ns3_testbed_simtime` 기능 마이그레이션**
  - [ ] 5.1 ns-3.45의 `realtime-simulator-impl.cc` 소스 코드를 분석하여 기존 `ns3_testbed_simtime`의 변경점(시뮬레이션 시간을 공유 메모리에 쓰는 로직)을 수동으로 재적용합니다.
  - [ ] 5.2 수정된 코드를 포함하여 ns-3.45 라이브러리를 소스 컴파일합니다.
  - [ ] 5.3 `ns3_testbed_simtime`의 C++ 코드가 새로 컴파일된 ns-3 라이브러리를 사용하여 빌드되는지 확인합니다.
  - [ ] 5.4 `shared_simtime_test`를 실행하여 공유 메모리를 통한 시간 동기화 기능이 정상 작동하는지 단위 테스트를 수행합니다.

- [ ] **6.0 통합 테스트 및 검증**
  - [ ] 6.1 업그레이드된 환경에서 `testbed_runner.py`를 `example1.csv` 시나리오로 실행하여 전체 테스트가 정상적으로 구동되는지 확인합니다.
  - [ ] 6.2 `ns3_testbed_gui`를 실행하여 지연 시간 데이터가 이전과 같이 정상적으로 수집되고 표시되는지 확인합니다.
  - [ ] 6.3 Gazebo 시뮬레이션 창에서 로봇이 정상적으로 움직이고, `cmd_vel` 토픽을 통해 제어했을 때 올바르게 움직이는지 확인합니다.
  - [ ] 6.4 `top` 또는 `htop` 같은 시스템 모니터링 도구를 사용하여 업그레이드 후 CPU 및 메모리 사용량이 이전 버전에 비해 크게 증가하지 않았는지 확인합니다.

- [ ] **7.0 최종 정리 및 문서화**
  - [ ] 7.1 업그레이드 과정에서 추가된 임시 파일이나 불필요한 주석을 제거합니다.
  - [ ] 7.2 프로젝트의 메인 `README.md` 파일에 변경된 ns-3와 Gazebo 버전 정보를 업데이트합니다.
  - [ ] 7.3 빌드 또는 실행 방법에 변경 사항이 있다면 관련 문서를 수정합니다.
