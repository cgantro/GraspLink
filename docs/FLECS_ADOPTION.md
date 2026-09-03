# Why PoseLink Uses Flecs

## Cube 하나를 그리던 Viewer

Viewer를 처음 만들 때 목표는 단순했다. GLFW로 창과 OpenGL context를 만들고, Cube 하나에
Model/View/Projection matrix를 적용해 화면에 보이게 하는 것이었다. 이 단계에서 `Renderer`가
Mesh, Shader, Texture를 직접 만들고 들고 있는 구조는 자연스러웠다. 그 resource를 쓰는 곳도
Renderer 하나였고, 그릴 Object도 하나였다.

Pose를 적용하면서 Cube의 위치와 회전도 Renderer로 전달했다. 이 역시 Cube가 하나일 때는 큰
문제가 없었다. `Render(const Pose&)`는 현재 Cube를 어디에 그릴지만 알면 됐다.

## 두 번째 Cube가 생기면서

ECS 구조가 실제로 필요한지 확인하기 위해 같은 Cube resource를 쓰는 CubeA와 CubeB를 만들었다.
둘은 geometry, shader, texture는 같고 위치만 다르다. 이때 기존 구조를 그대로 유지하면 Renderer가
두 Cube의 transform을 따로 보관하고, 두 번의 draw call을 직접 작성해야 한다. 다음 Object가 생길
때마다 Renderer의 멤버와 렌더링 코드가 함께 늘어나는 방식이다.

처음에는 `Scene`과 `SceneObject`를 직접 두는 방법도 있었다. 하지만 Object ID, 생성과 제거,
component 조합, 렌더링 대상 순회를 모두 프로젝트에서 관리해야 한다. 현재 Viewer가 필요한 것은
그 관리 기능이지, 별도의 Scene API를 설계하는 일이 아니었다.

## flecs를 넣은 위치

그래서 Object 관리 부분만 flecs로 바꿨다. `ViewerApp`이 `flecs::world`를 소유하고, CubeA와
CubeB를 entity로 생성한다. 위치·회전·크기는 `Transform`에, 그릴 resource는 `Renderable`에 둔다.
두 entity의 `Renderable`은 같은 `Mesh`, `Shader`, `Texture`를 공유한다.

Renderer는 더 이상 Cube를 소유하지 않는다. 매 frame `Transform`과 `Renderable`을 함께 가진
entity를 query하고, `Transform`의 model matrix와 `Renderable`의 기존 OpenGL resource로 draw call을
수행한다. 따라서 Renderer는 CubeA나 CubeB의 이름, 생성 순서, 개수를 알 필요가 없다.

world는 Renderer나 Window보다 먼저 파괴한다. world 안의 `Renderable`이 마지막 shared pointer를
해제할 때 Mesh, Shader, Texture도 함께 정리되는데, 이 시점에는 OpenGL context가 아직 살아 있어야
하기 때문이다. 이 수명 순서를 `ViewerApp::Shutdown()`에서 명시적으로 유지한다.

## 이번 변경으로 기대하는 것

새 렌더링 Object를 추가할 때 Renderer의 멤버와 draw call을 늘리지 않아도 된다. 필요한 component를
가진 entity를 world에 추가하면 Renderer query에 포함되고, entity 또는 component를 제거하면 다음
frame부터 렌더링 대상에서 빠진다.

flecs는 여기서 Object 생성·제거와 component 조회만 맡는다. Window, Camera, Mesh, Shader, Texture,
OpenGL resource 생성 방식은 바꾸지 않았다. network, pose estimation, model loading 같은 기능도 이번
도입 범위에 포함하지 않았다.
