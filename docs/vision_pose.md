# Vision pose

## 실제 입력 목표

Calibration과 runtime tracking을 분리합니다.

```text
[Calibration] ChArUco images → camera matrix K + distortion → assets/calibration/camera.yaml
[Runtime] Camera → single ArUco corners → solvePnP(IPPE_SQUARE) → rvec/tvec → position/quaternion
```

ChArUco는 다양한 시점의 corner를 이용해 camera intrinsic과 distortion coefficient를 구하는 데 사용합니다. Runtime은 크기를 meter로 알고 있는 single square ArUco marker를 사용합니다. 실제 marker 크기가 2D corner에 metric scale을 부여하므로 올바른 translation을 얻는 데 필수입니다. `SOLVEPNP_IPPE_SQUARE`는 평면 정사각형 marker에 맞는 방법입니다.

초기 필수 범위에는 ArUco Board, optical flow, 일반 객체 feature matching/RANSAC/ML을 넣지 않습니다. partial occlusion, 장거리 불안정, detection drop 또는 pose jitter가 실측될 때만 Board를 검토합니다.

## Synthetic source

첫 synthetic source는 `x=0.5 sin(t), y=0, z=2` meter와 Y축 일정 각속도 회전으로 구현합니다. 호출자가 전달한 `steady_clock` microseconds만 사용하면 결정론적 ground truth가 되어 Vision error를 섞지 않고 network/buffer/interpolation error를 측정할 수 있습니다. 이후 직선, 원과 급격한 방향 전환 trajectory를 선택 가능하게 확장합니다.

## 구현 상태

Vision source 관련 header/source는 직접 작성할 수 있도록 비어 있습니다. 구현 순서는 [Vision 구현 가이드](implementation/05-vision.md)를 따릅니다.
