# PoseLink

PoseLink는 원격 Vision Node에서 추정한 객체의 6DoF Pose를 UDP로 전송하고, OpenGL Viewer에서 실시간으로 재현하는 C++ 프로젝트입니다.

단순히 Pose를 화면에 표시하는 데 그치지 않고, 네트워크 지연·지터·손실·순서 변경이 원격 3D 시각화에 미치는 영향을 측정합니다. Timestamp 기반 Jitter Buffer와 보간을 적용하여 다음 질문에 답하는 것이 프로젝트의 핵심입니다.

> 최신성을 유지하면서 네트워크 지터로 인한 원격 3D 시각화의 불안정을 얼마나 줄일 수 있는가?

ArUco는 실제 6DoF Pose를 만드는 입력 수단이고 OpenGL은 결과를 보여주는 출력 수단입니다. 프로젝트의 중심은 두 지점 사이의 실시간 상태 데이터 파이프라인을 직접 설계하고 구현하는 것입니다.

## 전체 데이터 흐름

```text
                IPoseSource
                     │
          ┌──────────┴──────────┐
          │                     │
SyntheticPoseSource       ArUcoPoseSource
          │                     │
   Known Ground Truth       Real Camera
          └──────────┬──────────┘
                     ↓
                  PoseSample
                     ↓
         Binary Serialization / UDP
                     ↓
       Validation / Sequence Analysis
                     ↓
          Timestamped Pose Buffer
                     ↓
             LERP / SLERP
                     ↓
             OpenGL 3D Viewer
```

Synthetic Pose는 network와 interpolation 오차를 독립적으로 측정하는 데 사용합니다. 실제 입력은 ChArUco로 camera calibration을 수행한 뒤, 크기를 알고 있는 단일 ArUco marker와 `solvePnP`로 생성할 계획입니다.

## 핵심 설계

- C++17과 CMake 기반의 모듈식 구조
- 최신 상태를 우선하는 UDP 전송
- ABI와 endianness에 독립적인 64-byte custom binary protocol
- Sequence Number를 이용한 loss/reorder/duplicate 분석
- Timestamp를 이용한 packet age와 render timeline 관리
- 크기가 제한된 timestamp 기반 pose buffer
- 위치 LERP와 quaternion SLERP
- OpenCV camera 좌표에서 OpenGL 좌표로의 명시적 변환
- Synthetic ground truth와 network impairment를 이용한 정량 실험
- Immediate rendering과 buffered interpolation의 latency/stability 비교

## 목표 실행 프로그램

구현이 완료되면 `VisionNodeApp`과 `ViewerApp`을 각각 독립 process로 실행합니다. 아래 명령은 현재 동작하는 기능이 아니라 직접 구현할 목표 CLI입니다.

```bash
# Viewer
poselink_viewer --port 5000 --buffer-ms 30

# Synthetic Vision Node
poselink_vision_node --source synthetic --host 127.0.0.1 --port 5000 --rate 30
```

Windows에서는 중간 UDP proxy로 delay, jitter, loss와 reorder를 재현할 계획입니다. 이때 Vision Node는 Viewer가 아니라 proxy port로 전송합니다.

```bash
poselink_viewer --port 5000 --buffer-ms 30
poselink_net_proxy --listen 5001 --target-port 5000 \
  --delay-ms 50 --jitter-ms 20 --loss 1 --reorder 1 --seed 1
poselink_vision_node --source synthetic --host 127.0.0.1 --port 5001 --rate 30
```

`--loss`와 `--reorder`의 단위는 백분율입니다. Linux network impairment 실험에는 `tc netem`을 사용할 계획입니다.

## 빌드

요구 사항은 C++17, CMake 3.20 이상입니다. Viewer를 활성화한 최초 configure에서는 GLFW와 GLM을 가져오기 위한 네트워크 연결이 필요합니다.

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

현재 CMake는 보존한 graphics module만 구성합니다. 나머지 source/header는 학습용 빈 파일이며 [구현 가이드](docs/implementation/README.md)의 순서에 따라 target과 코드를 직접 추가합니다. OpenCV는 실제 camera pipeline 단계에서 선택 dependency로 추가합니다.

## 구현 로드맵

이 저장소는 학습을 위해 각 단계를 직접 구현하고 검증하는 방식으로 진행합니다.

1. GLFW/GLAD/GLM으로 빈 창과 OpenGL Cube 구현
2. `Pose{position, orientation}`로 Cube의 Model Matrix 제어
3. `SyntheticPoseSource → Pose → Cube` 로컬 경로 구현
4. `Synthetic Pose → UDP → Cube` 별도 process 경로 구현
5. sequence/timestamp와 baseline network metrics 추가
6. delay/jitter/loss/reorder를 재현하고 무보정 상태 측정
7. Timestamped Pose Buffer와 LERP/SLERP 구현
8. Immediate와 Buffered Interpolation 비교
9. ChArUco camera calibration 구현
10. ArUco detection과 `solvePnP` 구현
11. `ArUcoPoseSource`를 기존 UDP publisher에 연결
12. 최종 실험, buffer 정책 결정과 결과 문서화

첫 번째 마일스톤이 끝날 때까지 UDP와 OpenCV는 추가하지 않습니다. 첫 기술 목표는 `Pose{position, quaternion}` 값을 바꾸면 Cube가 정확하게 이동·회전하는 것입니다.

파일별 작성 순서는 [구현 가이드](docs/implementation/README.md)에서 확인할 수 있습니다. 설계 계약은 다음 문서에서 확인합니다.

- [아키텍처와 스레드 수명](docs/ARCHITECTURE.md)
- [UDP 바이너리 프로토콜](docs/protocol.md)
- [Viewer buffer와 보간](docs/viewer_interpolation.md)
- [Vision Pose와 calibration](docs/vision_pose.md)
- [좌표계와 MVP](docs/coordinate_system.md)
- [실험 계획](docs/experiments.md)
- [설계 결정과 범위](docs/decisions.md)
- [지원 플랫폼과 의존성](docs/platforms.md)

## 프로젝트 완료 기준

Synthetic Pose와 실제 ArUco Pose가 동일한 UDP 파이프라인을 통해 OpenGL 3D Viewer에 표시되고, delay·jitter·loss·buffer delay를 변화시킨 실험으로 Immediate Rendering과 Buffered Interpolation의 차이를 설명할 수 있으면 프로젝트를 완료한 것으로 봅니다.
