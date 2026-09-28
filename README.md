# GraspLink

현재 프로젝트는 OpenGL 화면, 카메라, Flecs 렌더 시스템과 기본 디버그 도형만 포함한다.
로봇 관절 계산, GLB importer, asset manifest, CAD 변환 파이프라인은 아직 포함하지 않는다.

## 구조

```text
apps/viewer       실행 초기화와 설정
modules/viewer    OpenGL 리소스와 렌더 시스템
modules/common    공용 타입
third_party/glad  OpenGL 함수 로더
```

## 로봇 제어 시뮬레이션 최종 목표

향후 로봇 제어는 렌더링 코드와 분리하고, 동일한 Control Core를 Simulation과 Real Hardware에서 재사용할 수 있는 구조로 확장한다.

- Hardware / Simulation Interface 분리
- Joint Angle Limit
- Joint Velocity / Acceleration Limit
- Rendering과 분리된 Fixed Control Loop
- E-Stop 확장
- Zero Offset 확장

세부 설계와 구현 우선순위는 [Robot Control Simulation Goals](docs/CONTROL_SIMULATION_GOALS.md)를 따른다.
HCR-12A의 관절 범위와 최대 속도 기준은 [HCR-12A + 2F-85 Simulation Specs](docs/HCR12A_2F85_simulation_specs.md)에 정리한다.

## 빌드

```powershell
cmake -S . -B build -DGRASPLINK_BUILD_GRAPHICS=ON
cmake --build build --config Release
```

실행 파일은 `grasplink_simulator`이며, 현재는 GLB 파일을 읽지 않는다.
