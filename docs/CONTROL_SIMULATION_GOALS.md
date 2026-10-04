# Robotics Simulation Goals

## 목표

렌더링 구현이나 특정 하드웨어와 결합하지 않은 동일한 robotics core를 Simulation과 Real Hardware에서 재사용한다.

```text
Motion Command
      ↓
IRobotController
      ↓
Backend
├─ Simulation
└─ Hardware
      ↓
RobotState
      ↓
Kinematics / Physics / Viewer
```

## 우선순위

1. **Backend 분리**
   - `IRobotController` / `IGripperController`를 공통 contract로 사용
   - `SimRobotController`와 향후 Hardware backend를 분리

2. **Joint Angle Limit**
   - model별 `minPositionRadians`, `maxPositionRadians`
   - 적용 전 target 검증

3. **Velocity / Acceleration Limit**
   - model별 최대 속도 사용
   - 목표 각도를 즉시 적용하지 않고 시간에 따라 상태를 변화
   - 제조사 최대 가속도 값이 확인되기 전에는 임의 상수를 사양으로 취급하지 않음

4. **Fixed Control Loop**
   - Rendering FPS와 제어 주기를 분리
   - accumulator 기반 고정 `dt`로 Controller를 Update

5. **Physics / Grasp**
   - Jolt Physics 사용
   - Flecs는 ECS/state, Jolt는 rigid body/collision/constraint 담당
   - 2F-85 contact 이후 under-actuated 동작은 physics/constraint 계층에서 처리

6. **Safety 확장**
   - software stop 상태
   - watchdog
   - E-Stop state model
   - zero offset

실제 E-Stop 회로를 software `Stop()`이 대체하지 않는다.

## Physics Integration

```text
Flecs Entity
├─ Transform
├─ RigidBody
├─ Collider
└─ PhysicsBodyHandle
        │
        ▼
   Jolt BodyID
        │
        ▼
 Jolt PhysicsSystem
```

Jolt 전용 API는 향후 `modules/physics` 내부에 격리한다.

## 구현 순서

```text
1. robotics/core + model/backend 분리
2. Joint angle/velocity limit
3. Fixed Control Loop
4. Robot Model / FK 검증
5. IK
6. Gripper runtime controller
7. Jolt Physics / collision
8. contact 기반 grasp / attach / detach
9. Watchdog / E-Stop state / Zero Offset
```
