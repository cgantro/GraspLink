# Architecture

## 모듈 경계

```text
modules/
├─ robotics/
│  ├─ core/                 Controller interface / state contract
│  ├─ models/               Robot/Gripper model constants
│  └─ backends/             Simulation / Hardware implementations
├─ viewer/                  OpenGL / Flecs / GLB and ECS integration
└─ physics/                 Jolt rigid-body simulation
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

## Physics와 Flecs 경계

`modules/physics`는 Jolt의 초기화, Body 생성/삭제, 고정 시간 Step과 pose 조회만 담당한다.
이 모듈은 Flecs, Viewer, OpenGL을 include하지 않고 Jolt 타입도 public API로 내보내지 않는다.
외부에는 `PhysicsWorld`, `PhysicsBodyHandle`, GLM 기반 물리 설정과 pose만 보인다.

```text
Entity.set<RigidBody>(...)
      .set<BoxCollider>(...)
          ↓
PhysicsSystemModule (Viewer / Flecs integration)
  - 설정 컴포넌트로 Body 생성
  - Entity 수명과 Body 수명 연결
  - Local / World Transform 변환
          ↓
PhysicsWorld (modules/physics)
  - opaque PhysicsBodyHandle 반환
  - Jolt Body 생성 / 삭제 / Step
          ↓
Jolt types (modules/physics 내부 전용)
```

`PhysicsSystemModule`은 고정 제어 루프에서 명시적으로 호출하는 ECS 통합 객체다. Flecs의
render-frame `progress()`에 물리 Step을 맡기지 않는다. 두 객체가 필요한 이유는 Jolt lifetime/API와
Entity 컴포넌트 해석·좌표 동기화가 서로 다른 책임이기 때문이다. 하나로 합치면 Physics 모듈이 ECS를
알거나 ViewerApp이 Jolt Body 생성과 Entity 동기화를 직접 떠안게 된다.

`PhysicsBodyBinding`은 같은 Entity에 붙는 private runtime component다. 게임 코드가 Body handle을
직접 생성·보관하지 않으며, Entity 삭제나 물리 설정 제거 시 observer가 Body를 제거한다.

### 좌표와 실행 순서

- Kinematic Body는 Entity 계층의 현재 Local Transform을 World pose로 계산해 Jolt에 전달한다.
- Dynamic Body는 Jolt World pose를 부모 World 행렬의 역행렬로 바꿔 Entity Local Transform에 쓴다.
- Box collider는 Entity 기준 local 위치·회전과 meter 단위 반 크기를 가진다. Entity scale은 Collider 크기에
  자동 반영하지 않는다. Robot Link와 Dynamic object는 unit scale을 사용하며 Floor는 render scale과 별도로
  collider 크기를 지정한다.
- Floor는 `plane.glb`를 렌더링하는 Entity 자체에 Static Box Collider를 가진다. Mesh를 Jolt shape으로
  변환하지 않는다.
- 로봇 J1~J6은 Controller가 pose를 정하는 Kinematic collision proxy다. 관절 제약/토크를 계산하는
  articulated dynamics는 아직 구현하지 않았다.

```text
FixedControlLoop
→ RobotController Update
→ RobotTransformAdapter (joint Local Transform)
→ PhysicsSystemModule (Entity → Kinematic Body)
→ PhysicsWorld Step
→ PhysicsSystemModule (Dynamic Body → Entity Local Transform)
→ Render frame: TransformSystem / RenderSystem
```

Entity 계층 행렬 계산은 `TransformSystemModule`의 공용 계산 함수를 쓴다. Physics가 fixed update에서
바뀐 최신 Local 값을 읽고, Renderer는 다음 render frame에 같은 Local 값으로 World matrix를 갱신한다.
