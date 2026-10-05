# Code Documentation Audit

## 목적

이 문서는 GraspLink의 public class/struct/interface를 전수 검토하면서 적용한 주석 기준과 범위를 기록한다.
단순히 클래스의 역할만 적는 것이 아니라, 코드를 처음 읽는 사람이 값의 의미와 변환 경계를 추적할 수 있도록 다음 정보를 우선한다.

- 값이 무엇을 뜻하는가
- 단위가 무엇인가 (`m`, `rad`, `rad/s`, `pixel`, `byte`, raw 0..255 등)
- 어느 좌표계/local frame의 값인가
- 소유권이 있는가, non-owning reference/pointer인가
- sentinel/default 값의 의미가 무엇인가
- 실제 장비/모델 수치가 runtime/graphics 표현으로 어디서 변환되는가
- 아직 구현되지 않은 값은 무엇이며, 임의 수치를 만들지 않은 이유가 무엇인가

## 공통 단위 규칙

Robotics core 내부:

```text
Joint angle        rad
Joint velocity     rad/s
Cartesian position m
Linear velocity    m/s
Angular velocity   rad/s
Time               s
Quaternion array   [x,y,z,w]
```

Graphics/asset 쪽은 source asset numeric unit을 임의로 바꾸지 않는다.
현재 controller-ready HCR-12A + Robotiq 2F-85 GLB는 meter 기준으로 정규화되어 있으므로 해당 모델의 vertex/translation은 meter다.

화면/API 계층의 대표 단위:

```text
Framebuffer/window size    pixel
Cursor delta               pixel
GPU buffer size            byte
Index count/offset         index element count
PBR factor                 dimensionless
Scale                      dimensionless
Gripper rPR/rSP/rFR        raw normalized 0..255
```

## 실제 로봇 수치 -> 그래픽 변환 경로

HCR-12A는 `models/hanwha/Hcr12a.h`에서 모델 정적 규격을 보관한다.

```text
제품/CAD 기준
  - joint pivot: mm
  - angle limit: deg
  - max velocity: deg/s
        |
        | specification 작성 시 정규화
        v
Hcr12a.h
  - pivot: m
  - angle: rad
  - max velocity: rad/s
  - axis: controller-ready GLB joint-local unit vector
        |
        v
SimRobotController
  - RobotState q: rad
  - RobotState dq: rad/s
        |
        v
RobotTransformAdapter
  q + local axis
  -> glm::angleAxis(q, axis)
  -> quaternion
  -> 현재 ECS 저장 형식인 Euler rad
        |
        v
Entity::SetLocalRotation
        |
        v
TransformSystemModule
  Local = Translation * Rotation * Scale
  World = ParentWorld * Local
        |
        v
RenderSystem / Renderer / Shader
```

관절 pivot은 controller-ready GLB Node translation/hierarchy에 이미 들어 있다.
따라서 `JointSpecification::bindPivotMeters`를 RobotTransformAdapter가 매 frame 다시 더하지 않는다.
이 값은 모델 규격, asset 검증, 향후 FK 기준으로 사용한다.

## Gripper 수치 의미

Robotiq 2F-85는 baked GLB animation을 runtime control source로 사용하지 않는다.

```text
positionRequest 0..255
        |
        | future SimGripperController
        v
master linkage angle q [rad]
        |
        v
mimic multiplier
        |
        v
6 linkage Joint local rotations
```

`positionRequest=0`은 fully open, `255`는 fully closed 의미지만 mm나 rad 자체가 아니다.
현재 `TwoF85.h`의 `0.7929 rad`는 free-space nominal closed master linkage angle이며 실제 motor shaft angle을 의미하지 않는다.
접촉 이후 under-actuated adaptive motion은 향후 Jolt Physics/contact/constraint 계층의 책임이다.

## 전수 점검 범위

### Robotics

- `modules/robotics/include/robotics/core/ControlTypes.h`
  - enum 각 값, command/state 모든 field, 단위/range/sentinel 의미 보강
- `modules/robotics/include/robotics/core/IRobotController.h`
  - 모든 public function의 입력 단위, 반환/상태 의미 보강
- `modules/robotics/include/robotics/core/IGripperController.h`
  - activation/reset/command/update의 backend 의미 보강
- `modules/robotics/include/robotics/models/ModelTypes.h`
  - Vec3/Axis3의 unit/context 규칙 명시
- `modules/robotics/include/robotics/models/RobotSpecification.h`
  - pivot/axis/angle/velocity 의미와 ownership 명시
- `modules/robotics/include/robotics/models/GripperSpecification.h`
  - master/mimic, pivot/axis/range 의미 명시
- `modules/robotics/include/robotics/models/hanwha/Hcr12a.h`
  - CAD mm -> m, deg -> rad, deg/s -> rad/s와 graphics 적용 경로 명시
- `modules/robotics/include/robotics/models/robotiq/TwoF85.h`
  - master q, pivot, local -Z axis, mimic sign, 0..255 의미 및 Physics 경계 명시
- `modules/robotics/include/robotics/backends/simulation/SimRobotController.h`
  - 모든 API/member 의미, 수명, 지원/미지원 범위 명시
- `modules/robotics/src/backends/simulation/SimRobotController.cpp`
  - velocity-limited integration 식과 epsilon/scale helper 의미 명시

### Viewer - Robotics/Transform

- `modules/viewer/include/viewer/robotics/RobotTransformAdapter.h`
- `modules/viewer/src/robotics/RobotTransformAdapter.cpp`
  - RobotState [rad] -> local-axis quaternion -> Euler [rad] -> Flecs Transform 경로 명시
  - pivot을 재적용하지 않는 이유, identity bind rotation contract 명시
- `modules/viewer/include/components/TransformComponents.h`
  - Position/Rotation/Scale/TransformMatrix 및 Local/World tag 의미와 단위 명시
- `modules/viewer/include/Entity.h`
  - Local/World 책임, 각 transform API 단위/수명 의미 명시
- `modules/viewer/include/components/RobotComponents.h`
  - legacy/viewer representation과 RobotSpecification source-of-truth 경계 명시
- `modules/viewer/include/components/GripperComponents.h`
  - legacy openness/direction의 비물리적 상태임을 명시

### Viewer - Asset Pipeline

- `modules/viewer/include/assets/GraphicsTypes.h`
  - 모든 struct/field를 Doxygen 형태로 재정리
  - asset unit을 Loader가 임의 변환하지 않는다는 규칙 명시
- `modules/viewer/include/assets/GltfLoader.h`
  - glTF quaternion `[x,y,z,w]` -> GLM -> Euler [rad], matrix decomposition, no-scale 규칙 명시
- `modules/viewer/include/assets/AssetManager.h`
  - CPU->GPU 변환, cache ownership, no-unit-conversion 명시
- `modules/viewer/include/assets/PrefabFactory.h`
  - NodeData -> Flecs Local Component mapping과 mesh-less Joint 보존 명시
- `modules/viewer/include/assets/ResourceID.h`
  - 기존 문서가 FNV-1a, invalid sentinel, ownership/cache 관계까지 충분히 설명하고 있어 내용 검토 후 유지

### Viewer - ECS/Scene/Rendering

- `modules/viewer/include/RenderContext.h`
- `modules/viewer/include/components/RenderComponents.h`
- `modules/viewer/include/scene/Scene.h`
- `modules/viewer/include/scene/SceneManager.h`
- `modules/viewer/include/systems/TransformSystemModule.h`
- `modules/viewer/include/systems/RenderSystemModule.h`
- `modules/viewer/include/graphics/Camera.h`
- `modules/viewer/include/graphics/OrbitCameraController.h`
- `modules/viewer/include/graphics/Renderer.h`
- `modules/viewer/include/graphics/Mesh.h`
- `modules/viewer/include/graphics/Material.h`
- `modules/viewer/include/graphics/Texture.h`
- `modules/viewer/include/graphics/Shader.h`
- `modules/viewer/include/graphics/IndexBuffer.h`
- `modules/viewer/include/graphics/VertexBuffer.h`
- `modules/viewer/include/graphics/VertexArray.h`
- `modules/viewer/include/graphics/MultiSampleFrameBuffer.h`
- `modules/viewer/include/graphics/ShadowMap.h`
- `modules/viewer/include/graphics/Window.h`
  - 함수 parameter/return, pixel/byte/index/scene-unit, GPU ownership과 lifetime 의미를 점검/보강

### Viewer - Physics / ECS Integration

- `modules/physics/include/PhysicsTypes.h`, `PhysicsWorld.h`
  - Jolt 경계, handle validity, World pose와 meter 단위 명시
- `modules/viewer/include/components/PhysicsComponents.h`
  - Motion Type의 상태 방향, collider 크기 단위와 Entity-local offset 명시
- `modules/viewer/include/systems/PhysicsSystemModule.h`
  - Flecs와 PhysicsWorld 사이의 책임 및 참조 lifetime 명시
- `modules/viewer/src/systems/PhysicsSystemModule.cpp`
  - Body 생성·삭제 방향, Kinematic 입력과 Dynamic 결과 흐름, World/Local 변환 이유 명시
- `modules/viewer/include/scene/EntityFactory.h`
  - Factory가 설정 Component만 붙이고 Jolt Body 생성은 시스템에 맡기는 경계 명시
- `docs/PHYSICS_ECS_INTEGRATION.md`
  - ownership, fixed-step 순서, 현재 지원 범위와 Gripper/Grasp 미구현 범위 기록

### Application

- `apps/viewer/ViewerApp.h`
  - Composition Root의 객체별 소유권, Controller -> Adapter -> Physics -> ECS -> Renderer 흐름과 fixed timestep 경계를 명시

## 기존 구현부 주석 점검

다음 구현 파일은 이미 내부 helper/변환 이유가 충분히 주석화되어 있어 코드 의미를 바꾸지 않고 유지하거나 필요한 부분만 보강했다.

- `modules/viewer/src/assets/GltfLoader.cpp`
  - accessor byte/stride, glTF index type 정규화, quaternion 순서, matrix decomposition 설명 존재
- `modules/viewer/src/systems/TransformSystemModule.cpp`
  - 공유 Local 조립 함수의 `T * R * S`, system의 `ParentWorld * Local` 계산 설명 존재
- `modules/viewer/src/graphics/Camera.cpp`
  - View matrix 의미, degree -> radian projection 변환 설명 존재
- `modules/viewer/src/graphics/OrbitCameraController.cpp`
  - pixel delta -> orbit/pan/zoom 계산식 설명 존재

## 주석 작성 원칙

새 타입/API를 추가할 때 다음을 따른다.

1. 이름을 다시 읽어주는 주석은 피하고 **왜 존재하는지**를 적는다.
2. 숫자 field에는 가능한 경우 **단위와 유효 범위**를 적는다.
3. 좌표값에는 **local/world/base/tool 중 어느 frame인지** 적는다.
4. `0`, `-1`, `nullptr`, `false`가 sentinel이면 그 의미를 적는다.
5. raw pointer/reference는 **ownership과 required lifetime**을 적는다.
6. 실제 장비 수치를 변환했다면 **원래 단위 -> runtime 단위와 변환 경계**를 적는다.
7. 미구현 기능은 동작하는 것처럼 설명하지 않고 `Unsupported`, `TODO`, `valid=false` 등의 현재 상태를 정확히 적는다.
8. Renderer/Viewer와 Robotics source-of-truth가 중복되면 어느 쪽이 authoritative한지 명시한다.
