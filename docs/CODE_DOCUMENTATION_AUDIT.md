# Code and documentation audit

이 문서는 이번 코드·문서 정리 결과를 찾기 위한 간단한 목차다. 구현 변경 범위, 검토 결과, 확인된 제한은 [REFACTOR_AUDIT_RESULTS.md](REFACTOR_AUDIT_RESULTS.md)를 참고한다.

후속 그리퍼 충돌 구성, 팔 충돌 단순화와 한글 Doxygen 주석 보강은 [GRIPPER_COLLISION_AND_COMMENT_RESULTS.md](GRIPPER_COLLISION_AND_COMMENT_RESULTS.md)에 기록한다.

위 결과 문서의 테스트 수와 구현 상태는 해당 작업 시점의 기록이다. 이후 연결된 2F-85 자유공간 Controller·기구학·GLB 자세 및 기존 proxy 갱신의 현재 계약·검증 범위는 [GRIPPER_RUNTIME_DESIGN.md](GRIPPER_RUNTIME_DESIGN.md)를 참고한다. 힘·접촉 시 정지·파지는 아직 구현하지 않았다.

물리와 Flecs 설정·수명·좌표 흐름은 [PHYSICS_ECS_INTEGRATION.md](PHYSICS_ECS_INTEGRATION.md), controller 계약은 [CONTROLLER_INTERFACE.md](CONTROLLER_INTERFACE.md), 구현 우선순위는 [CONTROL_SIMULATION_GOALS.md](CONTROL_SIMULATION_GOALS.md)에 정리한다.

로봇·그리퍼 수치와 구현 상태는 [HCR12A_2F85_simulation_specs.md](HCR12A_2F85_simulation_specs.md), 각 수치의 출처 구분은 [MODEL_DATA_PROVENANCE.md](MODEL_DATA_PROVENANCE.md)를 기준으로 한다.
