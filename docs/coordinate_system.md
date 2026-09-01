# 좌표계

모든 위치 단위는 meter이고 quaternion 배열 순서는 `(x,y,z,w)`입니다. OpenCV `solvePnP`의 `rvec`, `tvec`은 object/marker 좌표의 점을 camera 좌표로 옮기는 변환, 즉 `X_camera = R X_object + t`를 나타냅니다.

OpenCV camera 좌표는 +X 오른쪽, +Y 아래, +Z 전방입니다. 이 프로젝트의 OpenGL view 좌표 규약은 +X 오른쪽, +Y 위, camera 전방 -Z로 정합니다. 변환 행렬은 다음과 같습니다.

```text
C = diag(1, -1, -1, 1)
p_gl = C p_cv
R_gl = C3 R_cv C3^-1   (같은 좌표기저로 회전을 표현할 때)
```

단순히 `tvec` 부호만 일부 바꾸거나 OpenCV matrix를 OpenGL model matrix에 그대로 복사하면 안 됩니다. 실제 장면 설계에서는 marker pose를 model transform으로 쓸지, camera pose의 역변환을 view transform으로 쓸지 하나를 명시해서 사용해야 합니다.

- Model: object local 좌표를 world 좌표로 변환
- View: world 좌표를 camera/view 좌표로 변환
- Projection: view frustum을 clip 좌표로 투영
- 최종 변환: `clip = Projection × View × Model × local`

월드 축, marker 축, tracked object와 trajectory에 동일 규약을 적용합니다. handedness, 행렬 저장 순서와 곱 순서도 구현 시 GLM 규약에 맞춰 테스트해야 합니다. 현재 코드에는 `CoordinateConverter`, MVP 기반 3D object, depth test가 구현되지 않았으므로 위 내용은 구현 계약입니다.
