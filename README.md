# GraspLink

현재 프로젝트는 OpenGL 화면, 카메라, Flecs 렌더 시스템과 기본 디버그 도형만 포함한다.
로봇 관절 계산, GLB importer, asset manifest, CAD 변환 파이프라인은 포함하지 않는다.

## 구조

```text
apps/viewer       실행 초기화와 설정
modules/viewer    OpenGL 리소스와 렌더 시스템
modules/common    공용 타입
third_party/glad  OpenGL 함수 로더
```

## 빌드

```powershell
cmake -S . -B build -DGRASPLINK_BUILD_GRAPHICS=ON
cmake --build build --config Release
```

실행 파일은 `grasplink_simulator`이며, 현재는 GLB 파일을 읽지 않는다.
