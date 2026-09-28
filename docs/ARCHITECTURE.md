# Architecture

## 책임 경계

`ViewerApp`은 창, 카메라, Flecs world와 렌더 시스템을 조립한다.
`modules/viewer`는 GPU mesh, shader, material, camera와 draw 호출만 담당한다.
`modules/common`은 그래픽 모듈이 공유하는 기본 타입만 둔다.

현재 범위에는 로봇 kinematics, GLB/OBJ importer, asset manifest, CAD 변환기가 없다.
따라서 런타임에서 외부 좌표계 보정이나 로봇 전용 node 이름을 해석하지 않는다.

## 실행 흐름

1. ViewerApp의 실행 상수로 창과 카메라를 초기화한다.
2. OpenGL context와 Flecs render system을 초기화한다.
3. 디버그 cube 하나를 GPU resource로 만든다.
4. 매 프레임 world를 진행하고 render system이 cube를 그린다.

새 모델 형식이나 로봇 제어를 추가할 때는 기존 viewer 책임을 침범하지 않는 별도 모듈로
설계하고, 현재 최소 실행 구조를 먼저 깨뜨리지 않도록 한다.

## 향후 Robot Control 경계

로봇 제어가 추가되면 Controller가 GLB node나 Flecs render component를 직접 수정하지 않는다.
제어 결과는 공통 Joint 상태를 통해 시뮬레이션과 렌더링으로 전달한다.

```text
Controller
    ↓
JointCommand
    ↓
Limit / Safety
    ↓
IRobotHardware
    ├─ SimRobotHardware
    └─ RealRobotHardware
    ↓
JointState
    ↓
Kinematics / Simulation
    ↓
Rendering
```

필수 제어 경계는 다음과 같다.

- Hardware / Simulation 구현 분리
- Joint Angle Limit
- Joint Velocity / Acceleration Limit
- Rendering Loop와 Fixed Control Loop 분리

E-Stop과 Zero Offset은 기본 제어 구조 이후 확장한다.

세부 기준은 [CONTROL_SIMULATION_GOALS.md](CONTROL_SIMULATION_GOALS.md)를 따른다.
