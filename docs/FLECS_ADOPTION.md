# Why PoseLink Uses Flecs

## 1. 도입 배경

Viewer 초기 단계에서는 `Renderer`가 Cube resource와 draw call을 직접 관리해도 문제가 없었다.

하지만 PoseLink의 최종 Viewer/Simulator에는 다음 객체가 생긴다.

```text
Tracked Object
Robot Base
Robot Links
End Effector
Debug Axis
```

Renderer가 각 객체의 수명과 상태를 직접 들고 있으면 rendering code와 scene-state 관리가 섞인다.

그래서 PoseLink에서는 **Viewer object 관리와 반복 처리 규칙만 Flecs로 분리**한다.

---

## 2. 현재 실제 구조

현재 코드에서 `ViewerApp`이 `flecs::world`를 소유한다.

```text
ViewerApp
├─ Window
├─ Renderer
├─ Camera
└─ flecs::world
   ├─ RenderContext
   ├─ CubeA: Transform + Renderable
   ├─ CubeB: Transform + Renderable
   └─ RenderSystem
```

현재 주요 코드:

- `apps/viewer/ViewerApp.cpp`
- `apps/viewer/ViewerApp.h`
- `modules/viewer/include/RenderContext.h`
- `modules/viewer/include/components/Transform.h`
- `modules/viewer/include/components/Renderable.h`
- `modules/viewer/src/systems/RenderSystem.cpp`

---

## 3. `RenderContext`

`Renderer`와 `Camera`는 ECS component가 아니라 application-owned subsystem이다.

따라서 world context로 non-owning pointer를 전달한다.

```text
ViewerApp owns Renderer / Camera
        ↓
world.set<RenderContext>()
        ↓
RenderSystem reads RenderContext
```

이 구조 덕분에 `Renderer`가 Flecs를 include할 필요가 없다.

---

## 4. `RenderSystem`

현재 `RenderSystem`은 Flecs module/system으로 등록된다.

```text
world.import<RenderSystem>()
        ↓
RenderSystem constructor
        ↓
world.module<RenderSystem>()
        ↓
world.system<const Transform, const Renderable>()
```

실행 phase는 `flecs::PreStore`다.

매 frame:

```text
world.progress(dt)
        ↓
RenderSystem
        ↓
Transform + Renderable query
        ↓
RenderContext
        ↓
Renderer::Draw
```

Renderer는 entity 이름, 개수, 생성 순서를 알지 않는다.

---

## 5. 왜 모든 기능을 Flecs System으로 만들지 않는가

PoseLink는 ECS를 프로젝트 전체 framework로 강제하지 않는다.

적합한 경우:

```text
같은 component 조합을 가진 여러 entity에
동일 규칙을 반복 적용
```

예:

```text
Transform + Renderable
→ RenderSystem
```

반대로 특정 tracked object 하나의 Pose를 외부 source에서 갱신하는 초기 단계는 application update로 처리해도 충분하다.

UDP decoder, OpenCV pipeline, binary protocol, FK/IK solver 역시 일반 C++ module로 유지할 수 있다.

---

## 6. Robot 단계에서의 Flecs 역할

Robot model이 추가되면 Flecs의 가치가 커질 수 있다.

예상 scene:

```text
Robot Base
└─ Link1
   └─ Link2
      └─ Link3
         └─ End Effector
```

향후 검토 대상:

- `RobotLink` tag/component
- `Joint` data component
- `ChildOf` relationship
- hierarchy traversal

다만 FK 계산을 Flecs 내부에 먼저 맞추지 않는다.

우선 robot joint/link transform 계산을 일반 C++로 검증한 뒤, 계산된 world transform을 Flecs entity에 반영하는 식으로 경계를 유지한다.

---

## 7. Resource ownership

`Renderable`은 `Mesh`, `Shader`, `Texture`를 `shared_ptr`로 참조한다.

여러 entity가 동일 GPU resource를 공유할 수 있다.

```text
CubeA ─┐
       ├─ same Mesh / Shader / Texture
CubeB ─┘
```

`ViewerApp::Shutdown()`에서는 world를 먼저 reset한다.

이유:

```text
world component 파괴
→ Renderable shared_ptr 감소
→ GPU resource 파괴 가능
```

이 시점에 OpenGL context를 가진 `Window`가 아직 살아 있어야 한다.

---

## 8. 현재와 향후 기능 구분

| 기능 | 상태 |
|---|---|
| `flecs::world` ownership | 구현 |
| `RenderContext` world context | 구현 |
| `RenderSystem` module/system | 구현 |
| `world.progress(dt)` loop | 구현 |
| `Transform + Renderable` query | 구현 |
| Robot Link entity | 예정 |
| Flecs hierarchy / relationship | 검토 예정 |
| IK/FK system | 미정. ECS 종속 여부 결정 안 함 |

---

## 9. 판단 기준

Flecs를 사용하는 이유는 "ECS가 더 현대적이라서"가 아니다.

현재 선택의 목적은 다음 두 책임을 분리하는 것이다.

```text
Renderer
= OpenGL draw 책임

Flecs World/System
= 어떤 scene object를 반복 처리할지 결정
```

Robot 모델이 추가된 뒤에도 이 경계를 유지한다.
