# Viewer와 보간

핵심 비교 대상은 다음 두 정책입니다.

- Immediate: 가장 최근의 유효 pose를 즉시 표시해 latency를 최소화하지만 packet arrival 간격의 변동이 화면 움직임에 직접 나타납니다.
- Buffered interpolation: `renderTime = now - bufferDelay`로 두고 그 시점을 감싸는 pose A/B 사이를 보간합니다. 안정성이 좋아지는 대신 최소한 buffer delay만큼 표시 지연이 늘어납니다.

위치는 `p = (1-t)pA + tpB`인 LERP, 회전은 quaternion shortest-path SLERP를 사용합니다. quaternion dot product가 음수이면 한 quaternion 부호를 뒤집고, 거의 평행하면 정규화된 선형 보간으로 수치 불안정을 피합니다.

구현할 `PoseBuffer`는 timestamp순 삽입, 제한된 sample 수와 retention을 사용합니다. target time을 감싸는 두 sample이 있으면 보간하고, target이 범위 밖이면 가장 가까운 endpoint를 유지합니다. 초기 버전에서는 underrun 때 extrapolation하지 않습니다. 구현할 때 다음 정책을 명시해야 합니다.

- `valid=false` sample의 삽입 여부
- 같은 timestamp 두 sample의 처리
- stale/lost/disconnected threshold
- immediate와 buffered mode의 API/CLI 분리
- loss, jitter, FPS와 buffer depth 표시

목표 상태는 Vision의 `DETECTED → STALE → LOST`, 연결의 `CONNECTED → STALE → DISCONNECTED`이며 threshold는 실험 전에 명시해야 합니다. Prediction은 vision noise를 미분해 velocity noise를 키우고 motion model/filter 범위를 늘리므로 필수 범위에서 제외합니다.
