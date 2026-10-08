# 진단 도구

`grasplink_diagnostics`는 비동기 로그와 의미 지표를 기록한다. `Logger`는 각 기록을 크기가 제한된 큐에 복사하고, 작업자 스레드 하나가 기본 경로 `logs/grasplink.jsonl`에 JSON Lines 형식으로 쓴다. 큐가 가득 차면 제어 주기가 디스크 기록을 기다리지 않도록 새 기록을 버린다. 다만 Error 로그는 큐에 있던 metric 또는 Debug/Info 로그를 대신할 수 있다. `Flush()`는 이미 받은 기록을 모두 쓸 때까지 기다리고, `Shutdown()`은 새 기록을 받지 않고 큐를 비운 뒤 작업자 스레드를 종료한다. Logger를 사용하는 시스템이 모두 종료될 때까지 Logger를 유지해야 한다.

실행 중 성능은 Tracy로 확인한다. 시뮬레이터는 프레임 전체와 `Frame`, `FixedTick`, `RobotUpdate`, `CollisionCheck`, `PhysicsStep`, `Render` 구간을 기록한다. `TRACY_ON_DEMAND`를 켜면 Tracy Profiler가 연결될 때 기록을 시작한다. 시뮬레이터를 빌드하고 Tracy Profiler를 실행한 다음, 실행 중인 시뮬레이터에 연결해 각 구간의 시간을 살펴본다. Tracy CMake client는 `v0.14.1`로 고정되어 있다. Logger는 로그와 임무 결과 같은 의미 지표를 기록하며, 실행 시간 측정에는 사용하지 않는다.

다음 명령으로 고정 seed를 사용하는 경로 스트레스 벤치마크를 실행한다.

```powershell
build-ninja-release/grasplink_robotics_benchmarks.exe --benchmark_filter=RuntimeLinearPathStress
```

각 케이스의 TCP 목표는 관절 한계 안에 있는 관절값에서 FK로 만든다. `/64`와 `/256`은 한 번의 벤치마크 반복에서 생성하는 목표 수다. 결과에는 경로 계획 거절, 실행 중 Fault, 제한 시간 초과, 완료 수가 나온다. `planner_ik_limit_stalls`는 IK가 관절 경계에서 더 나아가지 못해 멈춘 횟수다. 이 값만으로 목표나 전체 경로가 도달 불가능하다거나, 시뮬레이션 중인 로봇 관절이 한계에 닿았다고 결론 내릴 수 없다. `planner_verified_joint_limit_violations`는 경로 관절값 검사에서 실제 범위 초과를 확인한 횟수다. `rejections_with_known_reachable_endpoints`는 실패한 경로 중 끝점이 합법 관절값에서 만들어져 도달 가능하다고 확인된 수다. 끝점에 도달할 수 있어도 그곳까지의 직선 TCP 경로 전체가 가능한지는 별도로 확인해야 한다.

이 벤치마크는 경로 계획과 Controller 실행을 분리해 검사한다. Jolt 환경 충돌, 그리퍼 파지 조건, 전체 Pick-and-Place 임무는 포함하지 않으므로 벤치마크 완료율을 임무 성공률로 해석하면 안 된다.
