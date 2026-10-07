# 리팩토링·주석 감사 결과

이 보고서는 최초 승인된 감사 작업의 완료 시점을 기록한다. 이후 그리퍼 충돌과 한글 Doxygen 주석 보강은 [후속 작업 보고서](GRIPPER_COLLISION_AND_COMMENT_RESULTS.md)에 기록한다.

이후 Local 회전 저장을 quaternion으로 전환해 GLB·FK·Physics의 Euler 왕복을 제거했다. 현재 회전 계약은 [Architecture](ARCHITECTURE.md)를 참고한다. 아래 double Euler 보강과 검증 숫자는 당시 결과로 보존한다.

기준: 2026-10-05 작업 시작 시의 실제 소스와 기존 미커밋 변경을 보존한 snapshot.
기존 simulation/gui/FK 구조를 유지하면서, 아래 경계 오류와 오래된 설명을 수정했다.
제조사 수치·CAD·GLB 자산은 변경하지 않았다.

## 주요 감사 항목과 처리

| 우선순위 | 문제 | 판단/구현 | 처리와 근거 |
| --- | --- | --- | --- |
| 높음 | Compound Body 무게중심을 이중 보정 | SOL | [PhysicsWorld](../modules/physics/src/PhysicsWorld.cpp): 공개 pose는 Body 원점, COM 변환은 Jolt 내부 처리. 비대칭 형상의 실제 낙하·접촉 테스트로 검증 |
| 높음 | 다른 PhysicsWorld의 같은 숫자 Body ID 사용 가능 | SOL | [PhysicsTypes](../modules/physics/include/PhysicsTypes.h): World token과 Jolt 세대 ID를 함께 검사 |
| 높음 | Link collider에 하위 관절·Gripper geometry 포함 | SOL | [RobotPhysicsAdapter](../modules/simulation/src/robotics/RobotPhysicsAdapter.cpp): 다음 가동 관절/Gripper에서 소유 범위를 끊음. 여러 hull의 전체 좌표 범위로 회귀 검증 |
| 높음 | FK와 GLB bind pivot/hierarchy 일치 여부 미검증 | SOL + LUNA | [RobotTransformAdapter](../modules/viewer/src/robotics/RobotTransformAdapter.cpp): bind 위치·행렬·관절 조상 검증. 실제 GLB ToolFrame과 FK 비교 |
| 높음 | J1 90°에서 quaternion → Euler 정밀도 손실 | SOL + LUNA | Viewer/Physics 어댑터 모두 double 정밀도로 Euler 변환 후 저장. GLB 비교 오차 약 0.316 mm에서 최대 약 0.000172 mm로 감소 |
| 높음 | Transform 없는 grouping parent가 조상 변환을 잃음 | SOL | [TransformSystemModule](../modules/viewer/src/systems/TransformSystemModule.cpp): 항등 Local로 조상 변환 전달, 파생 World cache 저장 |
| 높음 | Dynamic 조상 아래 물리 Body의 갱신 규칙 불명확 | SOL | [PhysicsSystemModule](../modules/simulation/src/systems/PhysicsSystemModule.cpp): 지원하지 않는 계층의 Body 생성/유지를 차단 |
| 높음 | Scene 활성화 전 생성/Scene 밖 재부모화로 소유 범위 이탈 | SOL + LUNA | [Scene](../modules/viewer/src/scene/Scene.cpp), [Entity](../modules/viewer/src/Entity.cpp): 활성 root·World·Scene·cycle 검사. 이름 없는 grouping 검색도 안전하게 처리 |
| 높음 | ModelResource GPU 공유 참조가 GL Context보다 오래 생존 | SOL + LUNA | [ViewerApp](../apps/viewer/ViewerApp.cpp): ModelResource까지 GPU 참조를 Context 종료 전에 해제. 실제 강한 공유 소유권으로 주석 정정 |
| 중간 | Shader/FBO 일부 생성 실패 시 자원 누수 | SOL + LUNA | [Shader](../modules/viewer/src/graphics/Shader.cpp), [MultisampleFramebuffer](../modules/viewer/src/graphics/MultisampleFramebuffer.cpp), [ShadowMap](../modules/viewer/src/graphics/ShadowMap.cpp): 실패 시 정리, resize 실패 후 같은 크기 재시도 지원 |
| 중간 | Grid의 25회 shadow 표본을 9로 나눔 | LUNA | [Grid shader](../assets/shaders/Grid.glsl): 25로 정규화 |
| 중간 | GLB sparse/range/matrix/hierarchy를 조용히 잘못 해석 | SOL + LUNA | [GltfLoader](../modules/viewer/src/assets/GltfLoader.cpp): 지원하지 않는 sparse, 범위/stride/overflow, 비유한 값, skew/perspective/퇴화 TRS, 순환·다중 부모·분리된 root 거부 |
| 중간 | Graphics OFF 의존성과 Ninja configuration/파일 대소문자 불일치 | LUNA | [Dependencies](../cmake/Dependencies.cmake), [Presets](../CMakePresets.json): 공통 GLM, Debug/Release 별도 경로, MultisampleFramebuffer 이름 통일 |
| 중간 | 임시 데모 동작이 NDEBUG에 따라 runtime에 섞임 | LUNA | [DebugSceneSetup](../apps/viewer/DebugSceneSetup.cpp): 앱에 분리하고 `--physics-demo`에서만 실행 |
| 중간 | GUI 표시를 실제 Jolt 형상으로 오해할 수 있음 | SOL + LUNA | [GuiModule](../modules/gui/src/GuiModule.cpp): ECS 설정의 X-ray 외곽선, 100 ms cache, 투영/clipping 한계 명시 |
| 낮음 | 중복 legacy component/spec와 사용되지 않는 marker 데이터 | LUNA + SOL | control의 구형 2F85 spec, viewer의 구형 Robot/Gripper component 제거. RobotCollisionProxy는 tag로 정리 |
| 낮음 | Doxygen/사전식 용어/오래된 데이터 흐름 주석 | LUNA | 책임·좌표·단위·수명·설계 이유 중심으로 수정. [COMMENT_GUIDELINES](COMMENT_GUIDELINES.md)에 Doxygen 금지 반영 |

## 병렬 작업과 통합 검토

- Luna 작업은 파일 소유 범위를 분리하고 외부 snapshot에서 수행했다. 같은 소스 파일을 동시에 수정하지 않았다.
- Scene, FK, loader, GPU, Physics, 자산, rendering, 입력/shader, 문서를 독립 작업으로 나눴다. 설계 판단 뒤 필요한 구현만 추가했다.
- 각 Agent가 자기 snapshot을 빌드·테스트했고, 주석 전용 변경은 주석/공백을 제거한 코드 비교로 동작 변경이 없음을 확인했다.
- Sol이 시작 snapshot과 최종 소스를 비교해 책임 경계, 중복 구현, 변경 충돌, 주석 내용을 재검토했다.
- 시작 snapshot과 비교한 52개 소스 파일은 주석/공백 변경만 있었다. 실행 코드가 바뀐 파일은 별도로 검토했다.
- 기존 build/test 산출물은 근거로 사용하지 않았다. 통합 검증은 최신 소스의 clean build로 수행했다.
- 소스의 Doxygen 태그·인코딩 손상·충돌 표식 검사, diff 공백 검사, 문서 12개의 local link와 code fence 검사를 통과했다.

## 회귀 검증

| Test | 검증 범위 |
| --- | --- |
| PhysicsWorld | 비대칭 COM의 실제 접촉, 원점 pose, Kinematic 이동, 삭제/재사용, 다른 World handle, 잘못된 pose |
| ControlRuntime | 4 ms 누적 tick, 시간 clamp, 명령 검증, 속도 상한, 재목표/Stop/Disconnect |
| RobotKinematics | 독립 기준점: zero/J1~J6/혼합 자세, ToolFrame, 잘못된 모델/상태 |
| GraphicsResources | 실제 GL 자원 ID의 해제, Shader compile/link/create 실패, FBO 실패와 resize 재시도 |
| GltfLoader | 현재 GLB 3개, 정상 packed/interleaved/matrix 입력, 잘못된 GLB 23개 |
| RobotPoseIntegration | 실제 GLB → GPU/Prefab → Transform → ToolFrame과 FK 위치·방향 비교, root 이동/회전, bind 불일치 |
| PhysicsEcsIntegration | grouping 부모, Dynamic feedback, collider 변경/삭제, Scene 전환, 소유 범위/계층 검사 |
| RobotCollisionGeometry | link geometry 소유 범위, FK proxy 위치, 잘못된 모델과 제거된 Scene |
| ViewerSmoke / ViewerDemoSmoke | 숨긴 GL Window에서 일반/데모 실행, Shader·GUI·렌더링·종료 경로 |

빌드·테스트 명령은 [README](../README.md)에 정리했다. Graphics OFF 구성에는 PhysicsWorld, ControlRuntime, RobotKinematics만 포함한다.

| 최신 소스 검증 구성 | 결과 |
| --- | --- |
| Graphics ON / Debug, clean build | 10/10 통과 |
| Graphics ON / Release, clean build | 10/10 통과 |
| Graphics OFF / Debug, clean build | 3/3 통과 |
| Graphics OFF / Release, 새 build directory | 3/3 통과 |

각 graphics 테스트는 이 Windows 환경의 실제 OpenGL Context에서 실행했다. Smoke 테스트는 숨긴 Window에서 8 frame을 실행하며, 육안으로 모든 렌더링 결과를 평가하는 테스트는 아니다.

## 유지한 설계 범위

아래 항목은 이 감사 당시의 비교 범위를 기록한 것이며 현재 기능 상태는 아니다. 최신 Robot/Gripper IK·접촉·환경 충돌 처리는 [로봇 이동과 파지](ROBOT_MOTION_AND_GRASP.md)와 [Physics/Flecs Integration](PHYSICS_ECS_INTEGRATION.md)을 참고한다.

- Controller의 관절 상태가 로봇 운동의 기준이다. FK가 공통 link/ToolFrame pose를 파생하며, Viewer와 Kinematic Physics proxy가 같은 결과를 사용한다.
- Dynamic 물체의 기준은 Jolt 결과다. 계산 뒤 부모 World 변환을 되돌려 Entity Local에 반영한다.
- FK ToolFrame은 모델의 고정 flange 기준이며 장착 공구 TCP feedback을 대신하지 않는다. SimRobotController의 `tcpPoseValid`는 false다.
- Collider는 GLB에서 만든 근사 convex hull이다. 오목한 부분을 메우거나 작은/평면 부품을 제외할 수 있다. Base/Gripper collider, 자기 충돌, 접촉 기반 grasp는 추가하지 않았다.
- IK/MoveLinear 실행, 가속도 제한, Gripper backend, 관절 torque/constraint/dynamics, hardware safety 기능은 별도 설계 작업으로 남긴다.
- GUI를 엔진 형상 검사기로 확장하거나 Viewer/Simulation 사이에 새 추상 계층을 넣는 대규모 변경은 하지 않았다.
