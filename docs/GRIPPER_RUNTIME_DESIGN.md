# 2F-85 개폐 제어와 접촉 파지

## 상태와 좌표의 기준

개폐 위치의 기준은 `IGripperController::GetState()`가 제공하는 `GripperState`다.
`closureFraction`은 무차원 연속 위치이며 0은 열린 기준, 1은 모델의 nominal closed 기준이다.
`closureFractionValid`와 전체 `valid`를 함께 확인한다. raw 0..255 위치는 요청·표시용이고,
연속 위치를 다시 8-bit로 반올림한 값을 관절 계산에 입력하지 않는다.

`GripperMode`는 연결·활성화·이동·정지를 나타낸다. `objectStatus`는 접촉 또는 목표 도달의 분류다.
중간 위치의 Stop은 `Stopped`, `goToActive=false`이며 도달하지 않은 목표를 도달했다고 보고하지 않는다.

흐름은 다음과 같다.

```text
GUI 요청 -> IGripperController
                |
                v
4 ms tick: Controller Update -> GripperState -> GripperKinematics
                                                |
                                                v
                                  GripperTransformAdapter
                                                |
                                                v
                     원본 GLB 관절 Local 회전 -> World 변환 갱신
                                                |
                                                v
                          기존 7개 Kinematic 프록시 -> Jolt step
```

Robotics는 Flecs·GLM·Jolt를 참조하지 않는다. GUI는 Controller 인터페이스에 요청을 보내며
Entity 자세를 직접 바꾸지 않는다. 앱이 Controller와 계산기의 수명, 고정 갱신 순서를 연결한다.
`ViewerApp`은 `GuiModule::BeginFrame/EndFrame` 사이에 `GripperPanel`을 호출한다. 패널은 요청 입력·마지막 결과만
보관하며 Controller는 Draw 동안만 빌린다. 물리 디버그 체크박스와 Collider 투영은 별도 패널·Overlay가 담당한다.

## 분기형 기구와 GLB bind

팔의 `RobotKinematics`는 직렬 체인의 base 기준 pivot을 누적한다. 그리퍼의 여섯 관절은 좌우 분기 구조이므로
이를 그대로 재사용하지 않는다. `GripperKinematics`는 master 각도와 mimic 계수에서 각 관절의 Local 회전
변화만 계산한다. 위치나 World 자세를 중복 생성하지 않는다.

현재 모델에서 master 각도는 `closureFraction * nominalMasterClosedRadians`이며 관절각은
`masterMultiplier * masterAngle`이다. 축은 관절 bind Local 기준이고 회전 형식은 모델의 [w,x,y,z]다.
nominal closed 값과 raw 위치의 선형 관계는 자유공간 시뮬레이션 근사이며 실제 장치 보정식이 아니다.

`GripperTransformAdapter`는 원본 Local 위치·크기를 보존하고 저장한 bind quaternion에 회전 변화를
오른쪽으로 곱한다: `bindRotation * deltaRotation`. 매번 저장된 bind에서 시작해 반복 적용의 누적 오차를 막는다.
Gripper root와 ToolFrame의 장착 변환도 그대로 둔다. root 기준 pivot을 중첩 관절의 Local 위치로 덮어쓰지 않는다.

bind와 delta를 정규화·결합한 quaternion을 `Entity::SetLocalRotation`으로 직접 저장한다.
`Rotation, Local`, `NodeData.rotation`, `ComposeLocalMatrix`는 quaternion을 사용하며 Euler 왕복 변환은 없다.
glTF `[x,y,z,w]`는 로더에서 GLM 생성자 `(w,x,y,z)`로 옮긴다. 영 quaternion·NaN 등 비유한 회전은 거부한다.
`q`와 `-q`는 같은 회전이므로 비교할 때 부호 차이를 허용한다. 모델 계산은 double, Entity pose와 행렬은 float다.
master/mimic 관절각은 rad 스칼라를 유지한다. Orbit 카메라의 마우스 입력용 yaw/pitch 각도는 별도 입력 계산이다.

실제 `assets/HCR12A_2F-85.glb`의 Gripper 노드에는 비항등 `matrix`가 있다. TRS 필드가 없다는 이유로
항등이라고 판단하면 안 된다. Tip 관절은 OuterKnuckle과 Finger 아래 중첩되어 있으며 부모 회전은
기존 TransformSystem의 계층 전파로 적용한다.

## Simulation backend 계약

`SimGripperController(specification, settings={})`는 사양을 빌리고 설정을 값으로 보관한다.
`SimGripperMotionSettings`의 기본 master 각속도 범위는 0.1..1.0 rad/s다.
이 값은 프로젝트의 시뮬레이션 기본값이며 제조사 속도 사양으로 취급하지 않는다.
raw speed는 사양의 최소..최대 범위를 위 각속도에 선형 대응한다. raw 0도 가장 느린 양의 속도로 움직인다.

- Connect: 연결되지 않은 상태에서 열린 위치로 초기화하고 Inactive로 진입한다. 이미 연결되어 있으면 현재 상태를 보존한다.
- Activate: 연결 상태에서 즉시 활성화한다. 이미 활성화되어 있으면 이동·위치를 유지한다.
- Command: 활성화가 필요하다. 위치·속도·힘 raw 범위를 먼저 검사하고 유효 요청만 목표로 교체한다.
- Update: 유한한 양의 dt에서 각속도 상한으로 연속 위치를 진행하고 목표에서 정확히 멈춘다.
- Stop: 현재 위치를 유지하고 Stopped로 전환한다. 요청 echo는 마지막 수락한 요청을 유지한다.
- Reset: 현재 위치를 유지하면서 이동을 중단하고 비활성화한다. 자동 열림 동작을 만들지 않는다.
- Disconnect: snapshot 유효성을 해제한다. 다시 Connect하면 열린 상태에서 시작한다.

`forceRequest`는 범위만 검사하고 접촉력과 전류를 계산하지 않으므로 `currentValid=false`다. 물리 계산 뒤 `GripperGraspAdapter`가 손끝 접촉을 전달한다. 한쪽 손끝만 닿으면 반대 손끝이 닫힐 때까지 계속 움직이고, 같은 물체를 양쪽에서 끼운 경우에만 닫힘을 멈춰 `ContactWhileClosing`을 보고한다. 열 때 손가락이 물체에서 멀어지는 접촉은 정지로 처리하지 않는다.

한쪽 접촉과 양쪽 파지를 구분한다. 양쪽 손끝이 같은 Dynamic 물체에 서로 반대 방향으로 닿으면 본체와 물체의 상대 자세를 유지하는 Jolt 고정 constraint를 만든다. 열기·Reset·Disconnect·Body 삭제나 교체에서는 해제한다. 개별 손가락 적응과 힘·마찰 안정성 계산은 지원하지 않는다. 자세한 원리와 앱 갱신 순서는 [로봇 이동과 파지](ROBOT_MOTION_AND_GRASP.md)를 참고한다.

## 검증

CPU 회귀는 연결·활성화·raw 범위·속도·dt·목표 교체·정지·reset·재연결·상태 복사,
모델 축·계수·한계와 회전 계산, invalid snapshot의 원자적 거부를 검증한다.
실제 GLB 통합 회귀는 bind 유지, 좌우 mimic, 중첩 tip World 자세, 반복 적용, 장착부 이동,
기존 7개 프록시의 수명과 움직임을 확인한다. 일반 Viewer와 물리 데모 숨김 실행도 유지한다.

2026-10-05 쿼터니언 저장·전달 전환 시점의 빌드·CTest 검증 결과다. 이후 GUI 분리·물리 동기화 변경의 최종 검증은 별도로 진행한다.

| Graphics | Debug | Release |
|---|---|---|
| ON | 15/15 | 15/15 |
| OFF | 5/5 | 5/5 |

`QuaternionTransforms`는 정규화·잘못된 입력 거부·q/-q·90도 부근 자세·계층 전파를 검증한다.
`GltfLoader`는 TRS와 matrix 양쪽의 quaternion 방향과 Local 행렬 보존을 확인한다.

같은 날 Physics 동기화와 GUI 분리 후 최종 검증은 Graphics ON Debug·Release 각각 16/16,
OFF Debug·Release 각각 5/5다. 추가한 `GuiComposition`은 네 형상의 표시·숨김과 Scene 교체,
패널 읽기 중 Controller 상태 보존을 확인한다. Physics 회귀는 Dynamic 부모 제한·복구,
Static 이동 후 접촉과 Kinematic 정지 뒤 선속도·각속도의 잔류 이동을 검사한다.
