# 설계 결정과 범위

## 확정 결정

- Vision Node와 Viewer를 별도 process로 분리해 실제 network 경계를 검증한다.
- `IPoseSource`로 deterministic synthetic ground truth와 실제 ArUco 입력을 교체한다.
- TCP의 순차 재전송보다 최신 pose 반영을 우선하므로 UDP를 사용한다.
- sequence는 전달 이상 탐지, timestamp는 temporal sampling에 사용한다.
- ABI 독립성과 versioning을 위해 custom binary serialization을 사용한다.
- 위치는 LERP, 회전은 quaternion SLERP로 보간한다.
- Immediate와 buffered interpolation을 모두 구현하고 latency/stability trade-off를 측정한다.
- Calibration은 ChArUco, runtime tracking은 우선 single ArUco와 `SOLVEPNP_IPPE_SQUARE`를 사용한다.

## 필수 완료 범위

```text
Synthetic pose + actual ArUco pose
→ custom UDP protocol and validation
→ sequence/timestamp analysis
→ timestamped jitter buffer
→ LERP/SLERP
→ OpenCV/OpenGL coordinate conversion
→ pose-driven OpenGL 3D visualization
→ netem/proxy impairment
→ immediate vs interpolation measurements
```

Prediction, optical flow, runtime ArUco Board, ROS2, Vulkan/DirectX/CUDA는 필수가 아닙니다. Protobuf/FlatBuffers는 제품 대안으로만 문서화합니다. “먼저 측정하고 실제 문제가 있을 때 개선한다”는 원칙을 따릅니다.

## 완료 판단 질문

코드와 측정 결과를 근거로 UDP 선택, loss/reorder 탐지, sequence/timestamp 역할, struct 직접 전송을 피한 이유, jitter의 화면 영향, buffer의 안정성/latency 교환, synthetic/ArUco 실험 분리, `solvePnP`, 좌표 변환, LERP/SLERP, 최종 buffer 정책과 가장 큰 한계를 설명할 수 있어야 완료입니다.

현재 repository는 graphics 영역만 보존하고 나머지 C++ 파일을 학습용 빈 골격으로 유지합니다. 파일별 구현 순서는 [구현 가이드](implementation/README.md)를 따릅니다.
