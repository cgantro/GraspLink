# Architecture

## Module boundaries

```text
modules/
├── robotics/       Robot models, controller contracts, backends, forward kinematics
├── physics/        Jolt wrapper and engine-independent physics types
├── viewer/         Flecs scene, transforms, GLB assets, OpenGL rendering
├── simulation/     Physics ECS integration, robot collision proxies, floor setup
└── gui/            ImGui panels and configured collider visualization
```

Arrows point from a consumer to its dependency. These are the main target dependencies in CMake:

```mermaid
flowchart TD
    App[ViewerApp] --> GUI[GUI]
    App --> Simulation[Simulation]
    App --> Viewer[Viewer]
    GUI --> Simulation
    GUI --> ImGui
    Simulation --> Viewer
    Simulation --> Robotics
    Simulation --> Physics
    Viewer --> Robotics
    Viewer --> Graphics[OpenGL graphics]
    Viewer --> Flecs
    Physics --> Jolt
```

`modules/physics` does not depend on Flecs, Viewer, or OpenGL. Jolt-specific types stay inside that module. `modules/simulation` owns Flecs physics configuration and the private runtime binding between an Entity and `PhysicsBodyHandle`.

## Robot state flow

```text
IRobotController
→ RobotState
→ RobotKinematics
→ RobotKinematicState
   ├── RobotTransformAdapter → GLB Joint Entity transforms
   └── RobotPhysicsAdapter → Kinematic link collision Entities
```

`RobotKinematics` is the shared source of robot pose. It derives parent-relative offsets from consecutive base-frame bind pivots and accumulates joint rotations for the serial chain. Viewer and Physics consume the same result instead of separately interpreting joint axes and angles. Robotics model data uses plain scalar/vector/quaternion types and does not depend on GLM, Flecs, or Jolt.

FK는 모델에 ToolFrame이 있으면 `toolFrameInBaseFrame`을 제공한다. 이 flange/model 기준은 `RobotState::tcpPose`와 별개이며 `SimRobotController`의 `tcpPoseValid`는 false다. IK, `MoveLinear` 실행, 가속도 제한, Hardware Gripper backend와 관절 동역학은 아직 구현하지 않았다.

`RobotTransformAdapter`는 자세를 authored GLB 계층에 적용한다. `RobotPhysicsAdapter`는 연결된 GLB 조각에서 링크별 Convex Hull을 만들며 다음 가동 관절 아래와 Gripper를 제외한다. 삼각형 중심 기준 16 cm 셀을 사용하고 4 cm 미만 부품·1 cm 미만 셀·부피 없는 hull 입력을 제외한다. `ConfigureTwoF85Colliders`는 고정 Gripper 또는 가동 관절 Entity 아래 Gripper-layer Kinematic proxy 7개를 만든다. rigid part마다 이름 있는 GLB 메시의 축약 hull을 사용하며 outer knuckle에는 attached finger 메시도 포함한다. proxy의 Local 원점은 소유 body/joint 원점이다. robot Base collider와 관절 동역학은 없다.

## 그리퍼 상태 흐름

```text
GUI 요청 → IGripperController / SimGripperController
→ GripperState.closureFraction
→ GripperKinematics: master angle / mimic Local rotation
→ GripperTransformAdapter: bind rotation * delta rotation
→ authored GLB 관절 → World 변환 → 기존 7개 Kinematic proxy
```

`GripperState`의 유효한 연속 위치가 개폐 자세의 기준이다. raw 0..255는 요청·표시용이며 반올림한 raw feedback을 기구학 입력으로 다시 사용하지 않는다. 여섯 관절은 분기형이므로 팔의 직렬 FK를 재사용하지 않고 Local 회전 변화만 계산한다. 어댑터는 원본 Local 위치·크기와 Gripper/ToolFrame 장착 변환을 보존한다. raw 위치의 선형 fraction 매핑과 master 속도 기본값 0.1..1.0 rad/s는 시뮬레이션 가정이다. 힘·전류·접촉 판정·접촉 시 정지·파지는 구현하지 않았다. 자세한 계약은 [그리퍼 런타임 설계](GRIPPER_RUNTIME_DESIGN.md)를 참고한다.

`modules/gui` owns the ImGui context and panels. Configured collider outlines come from ECS settings, not Jolt body inspection. They use X-ray screen projections without depth testing and refresh every 100 ms while enabled, including on first display or resize.

## Transform and fixed-step flow

`TransformSystemModule::UpdateWorldTransforms()`는 공통 Local-to-World 계산 경로다. 4 ms 고정 tick에서 두 Controller 상태와 팔·그리퍼 자세를 갱신한 뒤 World 행렬, Kinematic 동기화, Jolt step, Dynamic 결과의 Local 반영 순서로 진행한다. step 뒤 World 행렬을 다시 계산하며 렌더 직전에도 갱신한다.

`Rotation, Local`과 `NodeData.rotation`은 `glm::quat`이며 `Entity::GetLocalRotation` / `SetLocalRotation` 및 `ComposeLocalMatrix(position, rotation, scale)`도 quaternion을 사용한다. glTF 배열 `[x,y,z,w]`는 로더에서 GLM 생성자 `(w,x,y,z)`로 옮긴다. `Rotation` 생성과 행렬 합성은 회전을 검증·단위 정규화하며 영 quaternion·NaN·Infinity를 거부한다. `q`와 `-q`는 같은 회전이므로 성분 부호만으로 자세가 다르다고 판단하지 않는다.

팔·그리퍼 FK 결과는 quaternion으로 Entity에 전달하고, Physics의 Dynamic 결과도 부모 World 역행렬로 Local 자세를 복원한 뒤 quaternion으로 저장한다. GLB·FK·Physics에서 quaternion과 Euler 사이의 왕복 변환을 제거했다. Entity pose·변환 행렬은 float 정밀도를 유지한다. 제어용 관절각과 각속도는 rad·rad/s 스칼라이며, Orbit 카메라의 마우스 입력용 yaw/pitch도 rad 각도를 사용한다.

```text
FixedControlLoop
→ RobotController + GripperController Update
→ RobotKinematics + GripperKinematics
→ Viewer + Simulation pose adapters
→ TransformSystemModule World transforms
→ PhysicsSystemModule Entity-to-Physics
→ PhysicsWorld Step
→ PhysicsSystemModule Physics-to-Entity
→ TransformSystemModule World transforms 재갱신
→ Render frame: TransformSystemModule / RenderSystemModule
```

Physics uses fixed delta time and is independent of render FPS. Physics hierarchies require unit scale; only a Static Environment Entity's own visual scale is allowed. Collider dimensions and offsets are explicit meter values. Bodies with a Dynamic ancestor are rejected. Public body poses use the model/Entity origin; Jolt applies compound-shape center-of-mass offsets internally.

## Scene composition and lifetime

`ViewerApp` is the Composition Root. It creates Window, renderer, scene, controller, `PhysicsWorld`, adapters, `PhysicsSystemModule`, and `GuiModule` in dependency order. It owns no per-object creation APIs.

`SimulationSceneBuilder` configures the floor. Application-side `DebugSceneSetup` adds falling boxes and the robot demo command only with `--physics-demo`, in both Debug and Release. `PrefabFactory` builds visual Entities from GLB nodes. A single-primitive Node owns its render components directly; multi-primitive nodes use render child Entities. The GLB loader requires one node tree under the selected scene root and rejects sparse accessors, invalid byte ranges/strides, unsupported matrix transforms, cycles, and multiple parents.

`ViewerApp` owns Flecs World and PhysicsWorld. Physics integration objects and robot adapters are destroyed before those owners. Flecs removal observers delete the Jolt Body associated with a removed Entity.

Scene Entity creation requires an active SceneRoot. `SceneManager` creates the root before `OnEnter`, so constructors must defer Entity creation until activation. Shutdown removes adapters and Scene Entities while physics observers are live, then removes the integration objects and World. `ModelResource` also holds strong GPU Mesh references; it, AssetManager, shaders, and the renderer are released before the Window destroys the OpenGL context.

For more detail on collision layers, transform spaces, scale requirements, and deferred robot-physics work, see [Physics / Flecs Integration](PHYSICS_ECS_INTEGRATION.md).
