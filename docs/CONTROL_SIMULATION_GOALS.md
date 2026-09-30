# Robot Control Simulation Goals

## 목표

시뮬레이터의 제어 로직을 렌더링 구현이나 특정 하드웨어에 직접 결합하지 않고,
동일한 Control Core를 Simulation과 Real Hardware에서 재사용할 수 있는 구조를 목표로 한다.

```text
Target / Controller
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
```

## 우선순위

### 필수

1. **Hardware / Simulation Interface 분리**
   - 공통 `IRobotHardware` 정의
   - `SimRobotHardware`와 추후 `RealRobotHardware` 구현 분리
   - 상위 Controller는 구현체 종류를 알지 않도록 구성

2. **Joint Angle Limit**
   - Joint별 `minAngle`, `maxAngle`
   - 범위를 벗어난 명령을 적용 전에 검사

3. **Velocity / Acceleration Limit**
   - Joint별 `maxVelocity`, `maxAcceleration`
   - 목표 각도를 즉시 적용하지 않고 시간에 따라 제한된 상태 변화로 반영
   - HCR-12A 최대 속도는 사양 문서 값을 사용
   - 최대 가속도는 확인된 제조사 기준값이 없으면 별도 설정값으로 관리하고 임의의 제조사 사양처럼 취급하지 않음

### 목표

4. **Fixed Control Loop**
   - Rendering FPS와 제어 주기를 분리
   - 제어 루프는 고정된 `dt` 기준으로 Joint State를 갱신
   - 주기는 설정값으로 관리

### 확장

5. **E-Stop**
   - 활성화 시 신규 JointCommand 적용 중지
   - 제어 상태를 정지 상태로 전환
   - 실제 하드웨어의 안전 기능을 대체하는 것이 아니라 시뮬레이션용 상태 모델로 취급

6. **Zero Offset**
   - 논리적 Joint Zero와 센서/엔코더 기준점 사이의 차이를 보정할 수 있도록 offset 계층 추가

## Physics Simulation

물리는 직접 엔진을 구현하지 않고 **Jolt Physics**를 사용한다.
Flecs 자체에도 `flecs.components.physics`, `flecs.systems.physics` 계열이 있지만, 이 프로젝트의 3D rigid-body simulation backend로 사용하지 않는다.
Flecs는 Entity/Component/System과 상태 소유를 담당하고, Jolt는 rigid body, collision, constraint, physics step을 담당한다.

프로젝트에서 직접 구현하는 범위는 물리 엔진 자체가 아니라 **Flecs Entity / Robot State / Scene Transform과 Jolt Physics World를 연결하는 통합 계층**이다.

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

Flecs Entity가 Jolt Body 객체 자체를 소유하지 않고, ECS component에는 `BodyID` 같은 physics handle만 저장한다.

### 선택한 Physics Backend

- ECS: Flecs
- Physics Engine: Jolt Physics
- Language/Build 기준: C++17 + CMake
- Body type: Static / Dynamic / Kinematic
- ECS ↔ Physics 연결: Flecs component에 Jolt `BodyID` handle 저장
- Jolt 전용 API는 `modules/physics` 내부에 격리

### 필수 구현

- Jolt Physics 초기화 및 Physics World 생성
- Fixed Physics Step
- Dynamic / Static Rigid Body 구분
- 질량, 중력, 선형/각속도 상태 연동
- Robot/Environment용 Collision Shape 생성
- 바닥 및 물체 간 Collision Detection / Response를 라이브러리에 위임
- Flecs Transform ↔ Jolt Body Transform 동기화
- Grasp 시 Object를 End Effector에 연결
- Detach 시 Object를 다시 Dynamic Body로 전환하여 중력/충돌 적용
- Pick & Place 시 물체가 바닥과 다른 물체를 관통하지 않는지 검증

### 선택 확장

- Friction / Restitution 파라미터 조정
- Collision Layer / Mask
- Continuous Collision Detection
- Gripper 접촉 기반 grasp 판정
- Constraint 기반 grasp

직접 구현해야 하는 것은 Jolt Wrapper와 Flecs/Robot/Scene/Physics 간 데이터 흐름이며,
충돌 해결기나 rigid-body solver 자체를 새로 작성하지 않는다.

권장 모듈 경계:

```text
modules/physics
├─ PhysicsWorld
├─ RigidBody
├─ Collider
├─ PhysicsBodyHandle
└─ JoltPhysicsBackend
```

상위 Robot/Scene 코드는 Jolt API를 직접 호출하지 않고 physics module을 통해서만 접근한다.

## 기본 상태 / 설정

```cpp
struct JointState {
    double positionRad;
    double velocityRadSec;
    double accelerationRadSec2;
};

struct JointLimit {
    double minRad;
    double maxRad;
    double maxVelRadSec;
    double maxAccRadSec2;
};
```

내부 단위는 다음으로 통일한다.

```text
Angle        rad
Velocity     rad/s
Acceleration rad/s²
Time         s
```

## 구현 순서

```text
1. IRobotHardware / SimRobotHardware 분리
2. Joint Angle Limit
3. Velocity Limit
4. Acceleration Limit
5. Fixed Control Loop
6. Flecs + Jolt Physics 연동
7. Rigid Body / Collision / Ground 처리
8. Grasp Attach / Detach와 Physics 상태 전환
9. E-Stop
10. Zero Offset
```

Physics는 Jolt Physics를 사용해 Flecs와 통합하고, E-Stop과 Zero Offset은 기본 제어 구조와 제한 처리가 완료된 뒤 확장한다.


## 참고

- Flecs Hub는 physics/movement용 component와 system 모듈을 제공하지만, 본 프로젝트에서는 ECS 역할에 집중시킨다.
- Jolt Physics는 C++17 기반이며 CMake 통합을 지원하고 Static / Dynamic / Kinematic Body 구조를 제공한다.
