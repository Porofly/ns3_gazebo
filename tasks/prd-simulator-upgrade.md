# PRD: ns-3 및 Gazebo 시뮬레이터 버전 업그레이드

## 1. 개요 (Overview)

이 문서는 `ns3_gazebo` 워크스페이스에서 사용하는 핵심 시뮬레이션 엔진인 ns-3와 Gazebo의 버전을 업그레이드하는 과제에 대한 요구사항을 정의합니다.

- **현재 버전**: ns-3.29, Gazebo 9
- **목표 버전**: ns-3.45, Gazebo Harmonic

현재 버전은 오래되어 최신 네트워크 모델 및 물리 엔진의 개선 사항을 활용하는 데 제약이 있으며, 최신 ROS2 배포판과의 호환성 문제가 발생할 수 있습니다. 이 업그레이드를 통해 시뮬레이션 플랫폼을 현대화하여 개발 및 연구의 효율성과 정확성을 높이고자 합니다.

## 2. 목표 (Goals)

- ns-3.45 및 Gazebo Harmonic의 최신 기능을 프로젝트에서 활용할 수 있도록 통합합니다.
- 최신 ROS2 배포판(예: ROS 2 Jazzy)과의 호환성을 확보하여 미래 기술 스택에 대비합니다.
- 개발자에게 더 현실적이고 안정적인 시뮬레이션 시나리오를 구축할 수 있는 최신 개발 환경을 제공합니다.

## 3. 사용자 스토리 (User Stories)

- **개발자로서,** 최신 ns-3 및 Gazebo API를 사용하여 새로운 네트워크 모델과 개선된 물리 엔진을 활용함으로써 더 현실적인 시뮬레이션 시나리오를 구축하고 싶다.

## 4. 기능 요구사항 (Functional Requirements)

1.  워크스페이스 내 모든 `CMakeLists.txt` 파일을 수정하여 시스템에 설치된 ns-3.45 및 Gazebo Harmonic 라이브러리를 찾고 올바르게 링크해야 합니다.
2.  `src/`, `ns3_gazebo_plugin/`, `ns3_gazebo_ws/`, `ns3_testbed/`, `ns3_wifi_tap_test/` 등 모든 관련 프로젝트의 C++ 소스 코드를 ns-3.45의 API 변경사항에 맞춰 수정해야 합니다.
3.  `ns3_gazebo_plugin/`의 Gazebo 플러그인 코드를 Gazebo Harmonic의 플러그인 API (gz-sim) 변경사항에 맞춰 수정해야 합니다.
4.  기존의 Gazebo 월드 파일(`.world`)이 SDF(Simulation Description Format) 버전 차이로 인해 호환되지 않을 경우, 이를 Gazebo Harmonic에서 사용할 수 있도록 변환하거나 수정해야 합니다.
5.  모든 커스텀 ROS2 메시지(`.msg`)가 새로운 빌드 환경에서도 문제없이 생성되는지 확인해야 합니다.
6.  `testbed_runner.py` 스크립트와 Python 기반의 `ns3_testbed_nodes`가 업그레이드된 시뮬레이션 환경과 정상적으로 연동되어 작동하는 것을 검증해야 합니다.
7.  `ns3_testbed_simtime`의 시뮬레이션 시간 동기화 기능을 ns-3.45 버전으로 마이그레이션해야 합니다. 이는 정확하고 반복 가능한 성능 측정을 위해 필수적입니다.
    - `ns3_simtime_support/changed-ns-3.29-files/`에 있는 ns-3 소스 코드 변경점을 새로운 ns-3.45 소스 코드에 맞게 재적용해야 합니다.
    - `shared_simtime` 라이브러리가 새 환경에서 컴파일되고, `cpp_testbed_runner`가 이 라이브러리를 사용하여 시뮬레이션 시간 기준으로 올바르게 동작하는지 검증해야 합니다.

## 5. 제외 범위 (Non-Goals)

- **ROS2 배포판 업그레이드**: 이 과제는 시뮬레이터 버전 업그레이드에 집중하며, ROS2 배포판 자체를 업그레이드하는 작업은 포함하지 않습니다. (단, 호환성 확보는 목표에 포함됩니다.)
- **기능 변경**: 테스트 시나리오(CSV 파일)나 GUI(`ns3_testbed_gui`)의 기능을 변경하거나 확장하지 않고, 기존 기능이 동일하게 작동하도록 보장하는 데 집중합니다.

## 6. 기술적 고려사항 (Technical Considerations)

- **API 변경점 조사**: 작업의 첫 단계로 ns-3 (v3.29 -> v3.45) 및 Gazebo (v9 -> Harmonic)의 주요 API 변경점(breaking changes)을 반드시 조사해야 합니다. 특히 `TapBridge`, Wi-Fi 모듈, Gazebo 플러그인 API(`WorldPlugin` -> `System` 플러그인) 관련 변경점을 중점적으로 파악해야 합니다.
- **ns-3 소스 코드 수정**: `ns3_testbed_simtime` 기능의 핵심인 ns-3 시뮬레이터(`realtime-simulator-impl.cc`) 수정은 버전 차이로 인해 기존 패치를 그대로 적용할 수 없을 가능성이 높습니다. ns-3.45의 소스 코드를 분석하여 공유 메모리로 시뮬레이션 시간을 내보내는 로직을 수동으로 재구현해야 할 수 있습니다.
- **Gazebo (Ignition) 마이그레이션**: Gazebo 9에서 Gazebo Harmonic으로의 전환은 사실상 Ignition Gazebo로의 마이그레이션을 의미합니다. 기존 `gazebo::*` 형태의 API 호출을 `gz::sim::*` 또는 `ignition::gazebo::*` 형태로 전환하는 작업이 필요할 수 있습니다.
- **의존성 관리**: 모든 빌드 스크립트(`CMakeLists.txt`, `package.xml`)에서 라이브러리 이름, 의존성 패키지 이름 등이 변경되었을 수 있으므로 이를 확인하고 수정해야 합니다.

## 7. 성공 지표 (Success Metrics)

- **[컴파일]** 모든 C++ 코드가 ns-3.45 및 Gazebo Harmonic 라이브러리에 대해 경고나 에러 없이 성공적으로 컴파일된다.
- **[기능]** `ns3_testbed` 및 `ns3_testbed_simtime`의 기존 테스트 시나리오를 실행했을 때, 모든 테스트가 정상적으로 통과하며 이전과 유사한 성능 측정 결과를 보여준다.
- **[렌더링]** Gazebo 시뮬레이션이 시작되고 월드 및 로봇 모델이 깨짐이나 오류 없이 정상적으로 렌더링된다.
- **[통신]** `ros2 topic echo` 등의 명령어를 통해 로봇 노드 간의 통신이 ns-3 시뮬레이션 네트워크를 통해 정상적으로 이루어지는 것을 확인할 수 있다.
- **[성능]** 업그레이드 이후 시뮬레이션 시작 속도, 실행 속도, CPU/메모리 사용률 등 전반적인 성능이 이전 버전에 비해 현저히 저하되지 않는다.

## 8. 미해결 질문 (Open Questions)

- ns-3 v3.29에서 v3.45로 변경되면서 `TapBridge`, `WifiHelper`, `RealtimeSimulatorImpl` 등 이 프로젝트에서 사용하는 핵심 모듈의 구체적인 API 변경점은 무엇인가?
- Gazebo 9에서 Gazebo Harmonic으로 변경되면서 플러그인 API(예: `WorldPlugin`의 대체재)는 어떻게 변경되었으며, 마이그레이션에 필요한 주요 단계는 무엇인가?
- Gazebo Harmonic과의 완벽한 호환성을 위해 필요한 최소 ROS2 배포판 버전은 무엇인가?
