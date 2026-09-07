# Vision Pose

## 1. 목표

Vision 단계의 목적은 **원격 카메라에서 known object의 6DoF Pose를 생성하여 기존 UDP Object Pose pipeline에 넣는 것**이다.

```text
Camera
→ ArUco Detection
→ solvePnP
→ Object Pose
→ UDP Publisher
```

Vision Node는 robot IK/FK를 수행하지 않는다.

---

## 2. Calibration과 Runtime 분리

### Calibration — 예정

```text
ChArUco images
→ camera matrix K
→ distortion coefficients
→ calibration file
```

### Runtime Tracking — 예정

```text
Camera
→ ArUco corners
→ solvePnP
→ T_camera_marker
→ T_camera_object
→ Position + Quaternion
```

초기 object에는 marker를 부착하고 marker와 object frame의 상대 변환을 미리 알고 있다고 가정한다.

```text
T_camera_object
=
T_camera_marker
× T_marker_object
```

---

## 3. 왜 ArUco인가

현재 프로젝트에서 Vision 자체가 연구 주제가 아니다.

필요한 것은:

- 실제 camera 입력
- metric scale
- 6DoF pose
- 좌표계가 명확한 reference

이다.

Known-size ArUco marker는 이 요구에 맞으므로 첫 실제 입력으로 사용한다.

초기 범위에서 제외:

- markerless object detector
- feature matching 기반 일반 물체 tracking
- deep-learning 6DoF estimator
- multi-camera fusion

---

## 4. `solvePnP`

입력:

```text
known 3D marker corner coordinates
+
detected 2D image corners
+
camera intrinsic
+
distortion
```

출력:

```text
rvec
tvec
```

이 결과는 object/marker frame을 camera frame으로 옮기는 변환으로 해석한다.

평면 정사각형 marker에는 `SOLVEPNP_IPPE_SQUARE` 사용을 우선 검토한다.

---

## 5. Synthetic Source

Synthetic source는 Vision accuracy를 검증하기 위한 것이 아니라 **Vision 없이 Pose pipeline을 먼저 검증하는 ground truth source**다.

```text
Synthetic Pose
→ Viewer
```

이 경로가 정상 동작한 뒤:

```text
Synthetic Pose
→ UDP
→ Viewer
```

그 다음:

```text
ArUco Pose
→ UDP
→ Viewer
```

순서로 source만 교체한다.

---

## 6. Debug Preview

Vision Node의 정식 GUI는 만들지 않는다.

개발 중 detection 확인이 필요하면 OpenCV HighGUI를 optional preview로 사용할 수 있다.

표시 후보:

- raw frame
- detected marker corner
- marker ID
- pose axis

`ArUcoPoseSource` core 처리 안에 `cv::imshow()`를 직접 넣지 않고 application/debug-view 경계에서 표시한다.

---

## 7. Simulation으로 전달할 값

초기 단계에서 Vision Node가 보내는 것은 **object pose**다.

```text
Object Position
Object Orientation
```

Grasp Pose는 Viewer/Simulator가 계산한다.

즉:

```text
Vision Node
→ "물체가 어디에 있는가"

Simulator
→ "그 물체를 어디에서 어떤 자세로 잡을 것인가"
```

로 책임을 나눈다.

---

## 8. 현재 구현 상태

연결된 GitHub master 기준:

- `modules/vision/include/IPoseSource.h`: 빈 골격
- `modules/vision/include/SyntheticPoseSource.h`: 빈 골격
- `modules/vision/src/SyntheticPoseSource.cpp`: 빈 골격
- `modules/vision/src/ArUcoPoseSource.cpp`: 빈 골격
- OpenCV dependency: CMake에 아직 없음
- `apps/vision_node`: 파일은 있으나 빈 골격

프로젝트 진행 상태는 `Synthetic Pose → Cube` 완료로 관리하지만, 해당 구현은 현재 master snapshot에는 아직 반영되어 있지 않다.

---

## 9. ArUco 단계 완료 기준

- calibration file을 읽을 수 있음
- known marker가 검출됨
- `solvePnP`로 6DoF pose 생성
- marker 이동 방향과 position 변화 방향이 일치
- marker 회전 방향과 object orientation이 일치
- detection 실패 시 이전 유효 pose를 무조건 새 pose처럼 송신하지 않음
- 기존 Synthetic UDP pipeline과 같은 message path를 사용

수치 정확도 목표는 실제 camera/marker/조명 조건을 기록하고 baseline을 측정한 뒤 정한다.
