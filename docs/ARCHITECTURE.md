# 아키텍처

## 목표 데이터 경로

```text
IPoseSource
├─ SyntheticPoseSource (known ground truth)
└─ ArUcoPoseSource (real camera; planned)
        ↓ PoseSample
PacketEncoder → UDP Sender → network
        ↓
UDP Receiver → PacketDecoder/Validator → Sequence Analyzer
        ↓
Timestamped PoseBuffer → PoseInterpolator → CoordinateConverter
        ↓
OpenGL 3.3 Renderer
```

`IPoseSource`로 실제 Vision 오차와 network/interpolation 오차를 분리한다. Synthetic source는 정량 실험에, ArUco source는 실제 입력 pipeline 데모에 사용한다.

## 코드 구조

- `modules/common`: `PoseSample`, 단조 시계를 작성할 빈 파일
- `modules/transport`: protocol과 UDP socket을 작성할 빈 파일
- `modules/streaming`: receiver, metrics, buffer와 보간을 작성할 빈 파일
- `modules/vision`: synthetic/ArUco source를 작성할 빈 파일
- `modules/viewer`: 보존된 기존 OpenGL resource wrapper
- `assets/shaders`: Viewer가 runtime에 읽을 shader
- `apps/viewer`: Pose 수신·보간·렌더링을 조립하는 `ViewerApp` 실행 프로그램
- `apps/vision_node`: Synthetic/ArUco Pose source와 publisher를 조립하는 `VisionNodeApp` 실행 프로그램
- `tools/network`: network impairment 실험용 별도 도구

현재 CMake에는 `poselink_graphics`만 구성되어 있습니다. 나머지 target과 `poselink` 실행 파일은 [구현 가이드](implementation/README.md)의 순서에 따라 직접 추가합니다.

## 목표 동시성 및 수명 정책

Vision 쪽 목표 구조는 capture/vision producer와 UDP sender consumer 사이에 bounded queue를 두는 것입니다. 기본 overflow 정책은 `DROP_OLDEST`로 하여 오래된 pose 때문에 최신 상태가 밀리지 않게 하고 queue depth를 metric으로 기록합니다. Viewer에서는 network 수신이 pose buffer에 기록하고 OpenGL context 생성, 호출, 파괴는 render thread에만 귀속합니다.

목표 종료 순서는 입력 중지 → queue close → worker join → socket close → OpenGL resource/context 해제입니다. RAII로 socket, thread, GL resource lifetime을 관리하며 소유권이 하나면 `unique_ptr` 또는 값 타입을 우선합니다. 비그래픽 application과 thread/queue 코드는 현재 비어 있습니다.
