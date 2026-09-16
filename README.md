# GraspLink

GraspLink는 **외부 임베디드 장치에서 생성한 목표 위치를 네트워크로 전달하고, C++ 가상 로봇 시뮬레이터가 해당 위치로 이동해 물체를 파지하는 과정을 재현하는 프로젝트**다.

초기 프로토타입은 ESP32 + Zephyr RTOS를 외부 Target Controller로 사용한다. 버튼과 Potentiometer로 3차원 목표 좌표를 만들고 OLED에 표시한 뒤 UDP로 Simulator에 전송한다. Simulator는 Target Pose를 수신해 물체를 배치하고, Robot Model/FK/IK를 거쳐 End Effector를 목표 위치로 이동시킨 후 조건을 만족하면 Object Attach로 파지를 표현한다. 수행 상태는 다시 ESP32로 전송해 OLED/RGB LED/Buzzer로 표시한다.

Vision/ArUco 입력은 1차 완료 범위가 아니라 **동일한 Target Pose 인터페이스에 붙이는 2차 확장 입력원**으로 둔다.

## 프로젝트 목표

```text
ESP32 / Zephyr RTOS
  ├─ GPIO Interrupt : Button
  ├─ ADC            : Potentiometer
  ├─ I2C            : OLED
  └─ UDP Socket
          │
          │ Target Command
          ▼
C++ Simulator
  ├─ UDP Receiver
  ├─ Target Object
  ├─ Robot Model
  ├─ FK / IK
  ├─ Trajectory Update
  └─ Grasp / Object Attach
          │
          │ Robot State / Grasp Result
          ▼
ESP32
  └─ OLED / RGB LED / Buzzer
```

핵심은 단순 센서 데모가 아니라 **MCU 주변장치 → RTOS task/data flow → UDP → C++ simulator → robot kinematics → feedback**을 하나의 수직 경로로 연결하는 것이다.

## 4일 프로토타입 계획

| Day | 목표 | 완료 기준 |
|---|---|---|
| Day 1 | Zephyr bring-up + GPIO/I2C | ESP32에서 Zephyr 빌드/flash, 버튼 입력과 OLED 출력 확인 |
| Day 2 | Target Controller | ADC로 Potentiometer를 읽고 버튼으로 X/Y/Z 축을 선택해 Target Position 생성, task/message queue 구조 적용 |
| Day 3 | UDP 연동 | ESP32 → PC Target Command 송신, Simulator 수신 후 Target Object 위치 갱신 |
| Day 4 | Robot Grasp 수직 경로 | Robot Model/FK/IK → End Effector 이동 → Grasp 판정/Object Attach → 상태 ESP32 반환 |

4일 안에 범위를 넘기지 않는다. IMU, ArUco, network impairment 실험은 최소 프로토타입 이후에 확장한다.

## 개발 단계

1. **Synthetic Target Position → Object**
2. **Robot Model 추가**
3. **FK 구현**
4. **Grasp Pose 정의**
5. **IK 구현**
6. **End Effector → Target 추종**
7. **Grasp 성공 시 Object Attach**
8. **ESP32 + Zephyr 기본 환경 구성**
9. **Button / ADC / OLED 기반 Target Controller**
10. **ESP32 → Simulator UDP Target Command**
11. **Simulator → ESP32 상태 피드백**
12. **Thread / Message Queue 기반 임베디드 데이터 흐름 정리**
13. ArUco / Vision Target Source 추가
14. Network delay / jitter / loss 실험

1~12가 1차 프로토타입 범위다.

## 입력원 추상화 방향

Simulator는 Target이 어디서 왔는지 알 필요가 없게 구성한다.

```text
ITargetPoseSource
├─ SyntheticTargetSource
├─ UdpTargetSource        # ESP32
└─ ArucoTargetSource      # 향후 확장
```

따라서 로봇 제어 파이프라인은 입력원과 분리한다.

```text
Target Pose
→ Simulation Object
→ Grasp Pose
→ IK
→ FK
→ Robot Link Transform
→ Grasp / Attach
```

## Repository 구조

```text
apps/
└─ viewer/                  # C++ OpenGL/Flecs Simulator application

embedded/
└─ controller/              # ESP32 + Zephyr Target Controller
   ├─ CMakeLists.txt
   ├─ prj.conf
   ├─ boards/
   └─ src/

modules/
├─ common/                  # Pose / shared domain types
├─ transport/               # UDP / protocol
├─ streaming/               # receiver / latest-state handling
├─ vision/                  # Synthetic / future ArUco source
└─ viewer/                  # OpenGL / Flecs rendering

docs/
├─ ROADMAP.md
├─ EMBEDDED.md
├─ ARCHITECTURE.md
└─ ...
```

## 현재 상태

현재 repository의 실제 구현은 OpenGL/Flecs 기반 Simulator Viewer가 중심이다. 기존 `PoseLink` 이름과 Vision-first 계획을 **GraspLink + Embedded-first prototype**으로 전환하는 중이다.

구현되지 않은 기능은 완료된 것으로 간주하지 않는다. README의 계획은 목표 구조이며, 실제 완료 상태는 코드와 커밋을 기준으로 갱신한다.

## C++ Simulator 빌드

요구 사항:

- C++17 compiler
- CMake 3.20+
- OpenGL development environment
- 최초 configure 시 GLFW, GLM, Flecs를 가져올 네트워크 연결

```bash
cmake -S . -B build -DGRASPLINK_BUILD_GRAPHICS=ON
cmake --build build --config Release
```

현재 executable target 이름은 `grasplink_simulator`다.

## 설계 원칙

- Embedded firmware와 PC Simulator의 책임을 분리한다.
- MCU에서는 입력 처리, UI, 네트워크 송수신의 실행 책임을 분리한다.
- ISR에서 무거운 처리를 하지 않고 event/work/message queue로 넘긴다.
- Target Position과 Robot State를 별도 메시지로 구분한다.
- Simulator의 Robot/FK/IK 로직은 ESP32 구현을 직접 알지 않는다.
- FK correctness를 먼저 고정한 뒤 IK를 구현한다.
- 1차 grasp는 physics contact가 아닌 kinematic threshold + Object Attach로 제한한다.
- Vision은 동일한 Target Pose source 인터페이스의 확장으로 추가한다.
- 구현되지 않은 기능은 문서에서 `예정`으로 표시한다.

## 문서

- [4일 Roadmap](docs/ROADMAP.md)
- [Embedded Controller](docs/EMBEDDED.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Project Background](docs/PROJECT_BACKGROUND.md)
- [Build & Run](docs/build-and-run.md)
- [Protocol](docs/protocol.md)
- [Testing](docs/testing.md)
- [Acceptance Criteria](docs/ACCEPTANCE_CRITERIA.md)
