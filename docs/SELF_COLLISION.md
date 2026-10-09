# 로봇 자가 충돌 검사

`RobotEnvironmentCollisionGuard`는 후보 관절 자세의 FK와 각 proxy의 후보 World 변환을 계산해 환경 충돌과 로봇 자가 충돌을 검사한다. 후보 자세를 실제 Scene Entity에 적용하지 않으며, 두 프록시의 후보 변환을 Jolt shape 쌍 검사에 직접 전달한다. 따라서 Kinematic Body가 다음 물리 tick까지 이동하지 않은 상태에서도 Scene을 변경하지 않고 후보 자세를 검사할 수 있다.

허용 충돌 정책은 `RobotSelfCollisionPolicy.h`에 모여 있다. HCR-12A의 이웃 링크 쌍, Base와 Link1 베어링, Link6와 그리퍼 몸체 연결부만 허용한다. 나머지 링크 쌍은 겹치면 후보 자세를 거부한다. 일반 물리 접촉에서는 로봇 링크끼리 계속 제외하므로 시뮬레이션 접촉 신호와 접촉량은 바뀌지 않는다.

## 허용 쌍 빠른 참조

| proxy 쌍 | 후보 검사 정책 |
|---|---|
| 서로 인접한 HCR 링크 (`Link1`–`Link2` 등) | 허용 |
| Base–Link1 | 허용 |
| Link6–Gripper body | 허용 |
| 비인접 link pair, 그 외 Base/Gripper body pair | 금지; 겹치면 후보 자세를 거부 |
| Gripper finger / 내부 knuckle pair | 현재 검사 대상 아님 |

구체 판정은 `modules/simulation/include/simulation/robotics/RobotSelfCollisionPolicy.h`의 `IsAllowedSelfCollision()`가 수행한다. 후보 자세 계산과 실제 proxy 검사 흐름은 `RobotEnvironmentCollisionGuard`에 있고, 고정 tick에서의 복원 순서는 [Physics / Flecs Integration](PHYSICS_ECS_INTEGRATION.md)에 정리했다. 모델 좌표와 proxy 생성 기준은 [HCR-12A + 2F-85 Simulation Specification](HCR12A_2F85_simulation_specs.md)을 참고한다.

그리퍼 손가락과 내부 너클은 현재 자가 충돌 쌍에 포함하지 않는다. 이 부품들은 물체를 잡을 때 서로 가깝게 움직이고, 링크별 CollisionLayer만으로 의도된 내부 접촉과 장애물 접촉을 구분할 수 없다. 이 부품까지 포함하려면 관절 계층을 기준으로 한 추가 허용 쌍과 실제 그리퍼 자세의 회귀 검증이 필요하다.

회귀 테스트는 허용되는 이웃 링크, 검출 가능한 비이웃 링크 겹침, 그리고 HCR-12A 홈 자세에서 허용되지 않은 링크 쌍이 겹치지 않는 조건을 확인한다.
