# 진단 도구

`grasplink_diagnostics`는 비동기 로그와 의미 지표를 기록한다. `Logger`는 각 기록을 크기가 제한된 큐에 복사하고, 작업자 스레드 하나가 기본 경로 `logs/grasplink.jsonl`에 JSON Lines 형식으로 쓴다. 큐가 가득 차면 제어 주기가 디스크 기록을 기다리지 않도록 새 기록을 버린다. 다만 Error 로그는 큐에 있던 metric 또는 Debug/Info 로그를 대신할 수 있다. `Flush()`는 이미 받은 기록을 모두 쓸 때까지 기다리고, `Shutdown()`은 새 기록을 받지 않고 큐를 비운 뒤 작업자 스레드를 종료한다. Logger를 사용하는 시스템이 모두 종료될 때까지 Logger를 유지해야 한다.

실행 중 성능은 Tracy로 확인한다. 시뮬레이터는 프레임 전체와 `Frame`, `FixedTick`, `RobotUpdate`, `CollisionCheck`, `PhysicsStep`, `Render` 구간을 기록한다. `TRACY_ON_DEMAND`를 켜면 Tracy Profiler가 연결될 때 기록을 시작한다. 시뮬레이터를 빌드하고 Tracy Profiler를 실행한 다음, 실행 중인 시뮬레이터에 연결해 각 구간의 시간을 살펴본다. Tracy CMake client는 `v0.14.1`로 고정되어 있다. Logger는 로그와 임무 결과 같은 의미 지표를 기록하며, 실행 시간 측정에는 사용하지 않는다.

다음 명령으로 고정 seed를 사용하는 경로 스트레스 벤치마크를 실행한다. `/64`와 `/256`은 한 번의 반복에서 생성하는 목표 수다.

```powershell
build-ninja-release/grasplink_robotics_benchmarks.exe --benchmark_filter=RuntimeLinearPathStress
```

각 TCP 목표는 관절 한계 안의 관절값에서 FK로 만든다. 결과에는 경로 계획 거절, 실행 중 Fault, 제한 시간 초과와 완료 수가 나온다. label에는 마지막으로 거절된 경로의 TCP 표본 번호와 IK 실패 이유가 표시된다. `planner_ik_limit_stalls`는 IK 반복 중 관절 한계가 수렴을 막은 횟수다. 이 값만으로 전체 목표 경로나 실제 로봇 관절이 한계에 닿았다고 판단할 수 없다. `planner_verified_joint_limit_violations`는 검증한 경로 관절값에서 실제 한계 초과를 확인한 횟수다. `rejections_with_known_reachable_endpoints`는 실패한 목표의 끝점이 합법 관절값에서 만들어져 도달 가능하다고 확인된 수다. 끝점이 도달 가능해도 그곳까지의 직선 TCP 경로 전체가 가능하다는 뜻은 아니다.

`LinearPathPlanner`와 `IncrementalLinearPathPlanner`는 같은 고정 목표 경로를 각각 한 번에 계산하는 방식과 1.5 ms 프레임 예산으로 나눠 계산하는 방식으로 측정한다. 전체 경로 계획 시간은 Google Benchmark가 출력하는 `Time`과 `CPU`를 사용한다. `work_frames_per_path` 및 `estimated_latency_ms_at_60hz_*`는 증분 계획이 몇 프레임에 걸쳐 끝났는지와 60 Hz에서의 예상 대기 시간을 보여 준다. 프레임별 `slice_p50_us`, `slice_p95_us`, `slice_p99_us`, `slice_max_us`는 각 프레임에서 계획 계산에 걸린 시간을 별도로 측정한다. 한 작업 단위 자체가 예산을 넘으면 실제 slice 시간도 예산을 넘을 수 있다.

`ik_solves_per_path`, `ik_iterations_per_path`, `tcp_refinements_per_path`, `tcp_straightness_checks_per_path`와 `planner_state_checks_per_path`는 경로 하나를 만드는 데 사용한 계산량을 보여 준다. TCP 경로 오차는 각 관절 구간의 25%, 50%, 75% 지점에서 FK로 계산한 실제 자세와 요청 직선 경로의 차이를 측정한다. `max_fk_tcp_position_error_mm`와 `max_fk_tcp_orientation_error_mrad`가 그 최대 오차다. Incremental benchmark에는 실제 Jolt 충돌 검사기가 연결되지 않으므로 `planner_state_checks_per_path`는 상태 검사 호출 수일 뿐 충돌 쿼리 수가 아니다.

스트레스 벤치마크의 Controller에는 매 상태 검증 호출마다 횟수를 올리는 callback을 등록한다. `dummy_state_validity_callback_calls`는 callback 총 호출 수이고 `dummy_state_validity_callback_calls_per_case`는 목표 하나당 평균 호출 수다. Callback은 모든 상태를 유효하다고 반환하므로 실제 환경 충돌이나 Jolt 검사는 하지 않는다. 이 수치는 상태 검증 경계의 호출량을 나타내며 충돌 검사 횟수나 소요 시간을 뜻하지 않는다.

실제 Viewer에서 Tracy 캡처를 시작하면 충돌 검증 구간은 `CollisionCandidateFK`, `CollisionCheck`, `SelfCollisionCheck`, `JoltEnvironmentOverlap`, `JoltPairShapeOverlap`으로 나뉘어 표시된다. `CollisionCandidateFK`는 Scene을 수정하지 않고 FK에서 Jolt 입력 변환까지 만드는 시간이며, 두 Jolt 구간은 각각 환경 형상과 로봇 형상 쌍을 실제 검사한 시간이다. `SelfCollisionCheck`에는 허용 충돌 행렬과 각 쌍 반복 비용도 포함된다.

이 벤치마크는 경로 계획과 Controller 실행을 분리해 검사한다. Jolt 환경 충돌, 그리퍼 파지 조건, 전체 Pick-and-Place 임무는 포함하지 않으므로 벤치마크 완료율을 임무 성공률로 해석하면 안 된다.
