# Architecture

## 모듈 경계

```text
modules/
├─ robotics/
│  ├─ core/                 Controller interface / state contract
│  ├─ models/               Robot/Gripper model constants
│  └─ backends/             Simulation / Hardware implementations
├─ viewer/                  OpenGL / Flecs / GLB visualization
└─ physics/                 Jolt integration (future)
```

실제 파일은 public header를 `modules/robotics/include/robotics/...` 아래에 둔다.

## Robotics 데이터 흐름

```text
Application / Planner / IK
            ↓
      IRobotController
            ↓
       backend 구현
      ├─ simulation
      └─ hardware (future)
            ↓
         RobotState
            ↓
    RobotTransformAdapter
            ↓
       Flecs Joint Entity
            ↓
          Renderer
```

`modules/robotics`는 Viewer/Flecs/GLM에 의존하지 않는다.
`modules/viewer/include/viewer/robotics/RobotTransformAdapter.h`만 robotics domain state를 Viewer transform으로 변환한다.

## Model과 Backend 분리

`models/`는 장치의 고정된 기구/사양을 보관한다.

```text
models/
├─ RobotSpecification.h
├─ GripperSpecification.h
├─ hanwha/Hcr12a.h
└─ robotiq/TwoF85.h
```

여기에는 joint 이름, pivot, axis, angle limit, max velocity, gripper linkage 같은 model-specific 상수만 둔다.

`backends/`는 상태 변화와 외부 장치 연결을 구현한다.

```text
backends/
├─ simulation/
│  └─ SimRobotController
└─ hardware/               # future
   ├─ hanwha/
   └─ robotiq/
```

따라서 다른 6축/7축 로봇을 추가할 때 `IRobotController`를 다시 만들지 않고 새 `RobotSpecification`을 추가해 동일한 Simulation backend를 재사용할 수 있다.

## Viewer 책임

Viewer는 motion limit, trajectory, FK/IK를 계산하지 않는다.
`RobotTransformAdapter`는 `RobotState`의 관절 각도를 GLB Joint Entity local rotation에 적용하는 마지막 표현 계층이다.

## 향후 Physics

Jolt Physics는 별도 `modules/physics`에서 통합한다.
Flecs에는 Jolt 객체 자체가 아니라 `BodyID` 같은 handle만 저장한다.

```text
Flecs Entity
├─ Transform
├─ RigidBody
├─ Collider
└─ PhysicsBodyHandle
        ↓
   Jolt BodyID
```

Fixed Control Loop와 Physics Step은 Rendering FPS와 분리한다.
