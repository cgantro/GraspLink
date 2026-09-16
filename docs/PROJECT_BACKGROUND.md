# GraspLink Project Background

## 1. 프로젝트가 해결하려는 문제

GraspLink의 목표는 **외부 장치에서 지정한 공간상의 목표를 가상 로봇 시스템으로 전달하고, 로봇이 해당 위치의 물체를 찾아 이동·파지하는 수직 경로를 직접 구성하는 것**이다.

초기 시스템 흐름은 다음과 같다.

```text
Human Input
→ ESP32 Peripheral
→ Zephyr RTOS
→ Target Position
→ UDP
→ C++ Simulator
→ Simulation Object
→ Grasp Pose
→ IK / FK
→ Robot Arm
→ Grasp
→ State Feedback
→ ESP32 UI
```

단순한 센서 데모나 단순한 3D Viewer가 아니라 Embedded, Network, Graphics, Robotics 경계를 하나의 동작으로 연결하는 것이 핵심이다.

---

## 2. 왜 ESP32를 1차 입력원으로 두는가

기존 계획은 Camera/ArUco에서 Object Pose를 얻는 흐름을 먼저 구현하는 것이었다. 그러나 초기 prototype에서는 다음 문제가 동시에 섞인다.

```text
Camera calibration
Detection error
solvePnP error
Coordinate transform
Network
Robot kinematics
Rendering
```

그래서 1차 입력은 사용자가 직접 결정할 수 있는 Target Position으로 단순화한다.

```text
Button + Potentiometer
→ deterministic Target Position
→ UDP
→ Simulator
```

이렇게 하면 Embedded/RTOS/Network/Robot pipeline 자체를 먼저 검증할 수 있다.

Camera/ArUco는 이후 동일한 Target Pose interface에 추가한다.

---

## 3. 왜 Zephyr RTOS를 사용하는가

이 장치에는 서로 다른 책임이 존재한다.

```text
Input
Display
Network TX/RX
State Feedback
```

하나의 무한 loop에 모든 로직을 넣기보다 peripheral binding과 실행 책임을 분리하는 연습을 목표로 한다.

특히 다음 항목을 실제 코드에서 다룬다.

- DeviceTree 기반 board/peripheral binding
- GPIO interrupt
- ADC
- I2C
- thread/work/message queue
- UDP socket

단, RTOS를 사용한다는 이유로 thread를 무조건 늘리지는 않는다. 실행 주기와 blocking 특성이 다른 책임이 생길 때 분리한다.

---

## 4. 왜 Target Position과 Robot 제어를 분리하는가

ESP32는 Robot의 joint angle이나 IK 알고리즘을 알 필요가 없다.

```text
ESP32
→ "이 위치를 목표로 사용"
→ Simulator
```

Robot Model/FK/IK/Grasp는 C++ Simulator가 책임진다.

이 경계를 유지하면 Target source를 나중에 바꿀 수 있다.

```text
Synthetic
ESP32
ArUco Camera
      ↓
Target Pose
      ↓
Robot Pipeline
```

---

## 5. 왜 양방향 통신을 구성하는가

Target Command만 보내면 Embedded 장치는 명령을 보낸 뒤 실제 수행 결과를 알 수 없다.

따라서 Simulator가 상태를 반환한다.

```text
ESP32 → TARGET → Simulator
ESP32 ← STATE  ← Simulator
```

예정 상태:

```text
IDLE
TARGET_RECEIVED
MOVING
GRASP_SUCCESS
GRASP_FAILED
```

ESP32는 OLED/RGB LED/Buzzer로 결과를 표현한다.

이를 통해 단순 sender가 아니라 외부 Controller와 Simulator 사이의 상태 흐름을 구성한다.

---

## 6. 왜 FK를 IK보다 먼저 구현하는가

IK의 결과가 올바른지 검증하려면 joint angle에서 실제 End Effector Pose를 계산할 수 있어야 한다.

```text
Joint Angles
→ FK
→ End Effector Pose
```

이 기준을 먼저 확보한 뒤:

```text
Target Grasp Pose
→ IK
→ Joint Angles
→ FK
→ Error 확인
```

순서로 구현한다.

---

## 7. Grasp 범위

1차 prototype에서는 복잡한 physics/contact simulation을 하지 않는다.

```text
End Effector가 Grasp Pose 허용 오차에 진입
→ Grasp Success
→ Object Attach
```

즉 kinematic grasp를 사용한다.

초기 목표는 collision-free motion planning이나 dynamics가 아니라 **입력부터 파지 결과까지 전체 경로를 완성하는 것**이다.

---

## 8. 왜 OpenGL/Flecs를 유지하는가

Simulator는 로봇 link, target object, end effector, debug geometry를 직접 시각화해야 한다.

OpenGL을 사용해:

```text
Pose
→ Transform
→ Model Matrix
→ Renderer
```

경로를 코드 수준에서 확인한다.

Flecs는 scene entity/component와 render system scheduling에 사용하고, FK/IK 계산 자체는 rendering layer와 분리한다.

---

## 9. 1차 완료 범위

1. Synthetic Target Position → Object
2. Robot Model
3. FK
4. Grasp Pose
5. IK
6. Robot target 추종
7. Object Attach
8. ESP32 + Zephyr bring-up
9. GPIO interrupt / ADC / I2C
10. Target Position UI
11. ESP32 → Simulator UDP
12. Simulator → ESP32 상태 feedback

---

## 10. 1차 범위에서 하지 않는 것

- IMU 기반 위치 추정
- ArUco / camera pose estimation
- 일반 물체 detector
- neural network grasp planning
- collision-free motion planner
- robot dynamics / torque control
- physics gripper contact
- ROS2
- network impairment 실험

이 항목은 prototype 이후 확장한다.

---

## 11. 이후 확장

2차 단계에서는 Camera/ArUco를 Target Source로 추가한다.

```text
Camera
→ ArUco
→ Object Pose
→ Target Pose interface
→ 기존 Robot Pipeline
```

그 다음 delay/jitter/loss 환경에서 target age, packet loss, end-effector error, grasp result를 측정한다.

---

## 12. 프로젝트 한 문장

> GraspLink는 ESP32/Zephyr 기반 외부 Target Controller에서 생성한 목표 위치를 UDP로 C++ 로봇 시뮬레이터에 전달하고, 가상 로봇팔이 FK/IK를 이용해 목표 물체를 파지한 뒤 수행 상태를 다시 임베디드 장치에 반환하는 Embedded-to-Simulator robotic grasp 프로젝트다.
