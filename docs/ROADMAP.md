# GraspLink 4-Day Prototype Roadmap

## Goal

4일 동안 완성할 최소 수직 경로는 다음과 같다.

```text
ESP32 Input
→ Zephyr RTOS
→ Target Position
→ UDP
→ C++ Simulator
→ Robot IK/FK
→ End Effector 이동
→ Grasp / Object Attach
→ UDP State Feedback
→ ESP32 UI
```

이 기간에는 기능 수를 늘리기보다 **한 경로를 실제로 끝까지 연결하는 것**을 우선한다.

---

## Day 1 — Zephyr bring-up

### 목표

ESP32에서 Zephyr 애플리케이션을 빌드하고 flash할 수 있는 개발 환경을 고정한다.

### 작업

- VS Code + Zephyr/west 환경 확인
- 정확한 ESP32 board target 확인
- `embedded/controller` 빌드
- serial log 출력
- Button GPIO interrupt
- OLED I2C 출력
- board-specific devicetree overlay 작성

### 완료 기준

- 보드 부팅 로그 확인
- 버튼 입력 이벤트 확인
- OLED에 고정 문자열 표시

---

## Day 2 — Target Controller

### 목표

외부 장치만으로 Target Position을 만들 수 있게 한다.

### 작업

- Potentiometer ADC 읽기
- Button으로 X/Y/Z 선택
- ADC 값을 simulator coordinate range로 mapping
- OLED에 선택 축과 X/Y/Z 표시
- 입력 처리와 UI 갱신 책임 분리
- ISR에서는 event만 전달하고 실제 처리는 thread/work context에서 수행
- message queue 또는 event 구조 적용

### 완료 기준

```text
X: 0.30
Y: 0.10
Z: 0.45
```

형태의 목표 좌표를 ESP32에서 조작할 수 있다.

---

## Day 3 — Embedded ↔ Simulator UDP

### 목표

ESP32에서 생성한 Target Position이 PC Simulator의 Object 위치를 실제로 바꾸게 한다.

### 작업

- Target Command packet 정의
- sequence number 추가
- ESP32 UDP sender
- C++ UDP receiver
- 최신 Target만 적용
- Synthetic source와 UDP source 경계 분리
- 수신된 Target Position을 simulation object에 반영

### 최소 packet

```text
message_type
target_id
sequence
x
y
z
```

### 완료 기준

Potentiometer/Button으로 좌표를 변경하고 SEND했을 때 PC 화면의 Target Object 위치가 변경된다.

---

## Day 4 — Robot Grasp Vertical Slice

### 목표

수신한 목표 위치를 로봇 동작과 파지까지 연결한다.

### 작업

- Robot Model
- FK
- Grasp Pose
- IK
- 프레임 기반 joint/EE 추종
- 위치/회전 오차 기반 grasp 판정
- Object Attach
- Simulator → ESP32 상태 packet
- OLED/RGB LED/Buzzer 피드백

### 상태 예

```text
IDLE
TARGET_RECEIVED
MOVING
GRASP_SUCCESS
GRASP_FAILED
```

### 완료 기준

```text
ESP32에서 Target 입력
→ UDP 전송
→ 가상 Robot 이동
→ Object grasp
→ ESP32에 성공 상태 표시
```

가 한 번의 데모 흐름으로 동작한다.

---

## 4일 동안 제외

다음 항목은 프로토타입 완료 후 추가한다.

- IMU
- ArUco / Camera
- 일반 물체 인식
- collision avoidance motion planning
- physics-based gripper contact
- ROS2
- network jitter/loss 실험
- 복잡한 telemetry/dashboard

---

## Prototype 이후 확장 순서

1. ArUcoTargetSource
2. Quaternion/orientation target
3. Network delay/jitter/loss injection
4. target age / packet loss / grasp error 측정
5. 필요 시 IMU orientation controller
6. 센서/상태 telemetry 확장

---

## 포트폴리오에서 설명할 수 있어야 하는 질문

- 왜 Arduino loop가 아니라 Zephyr를 사용했는가?
- GPIO interrupt와 polling의 차이는 무엇인가?
- ISR에서 모든 로직을 처리하지 않은 이유는 무엇인가?
- DeviceTree가 어떤 하드웨어 정보를 분리하는가?
- 입력/UI/network task를 왜 분리했는가?
- UDP를 선택한 이유와 packet loss 시 정책은 무엇인가?
- stale target을 어떻게 처리하는가?
- Object Pose와 Grasp Pose는 어떻게 다른가?
- FK와 IK를 왜 분리해서 구현했는가?
- Embedded node와 Simulator 사이 책임 경계를 어떻게 잡았는가?
