# 실험 계획

정량 실험은 `SyntheticPoseSource`의 known ground truth를 사용하여 Vision noise와 network/interpolation error를 분리합니다. 초기 latency 실험은 `steady_clock`을 비교할 수 있는 동일 머신의 독립 process에서 수행합니다. sender의 `sleep_for`는 pose rate 제어에만 쓰고 impairment 모사는 Linux `tc netem` 또는 Windows UDP proxy를 사용합니다.

## 독립 변수

| 변수 | 값 |
|---|---|
| one-way delay | 0, 20, 50, 100 ms |
| jitter | 0, 5, 20, 50 ms |
| loss | 0, 1, 5, 10 % |
| reorder | 0, 1, 5 % |
| buffer delay | 0, 10, 20, 30, 50, 100 ms |
| pose rate | 15, 30, 60, 120 Hz |

전체 Cartesian product를 무조건 수행하지 않고 기준 profile(30 Hz, delay 50 ms, jitter 20 ms, loss 1%)에서 한 변수씩 바꾸며, 핵심 조합만 반복합니다. 난수 seed, 실행 시간, warm-up, 반복 횟수, OS/build 정보와 proxy/netem 설정을 raw result에 함께 저장합니다.

## 측정값

- sender: pose rate, queue depth/drop(구현 후), packet size
- Vision: capture FPS, detection/pose estimation time, failure rate(ArUco 구현 후)
- receiver: RX rate, invalid, loss/reorder/duplicate, arrival interval, jitter
- Viewer: render FPS, buffer depth, underrun, stale count/duration
- 품질: display latency, position error `||p_gt(t_render)-p_rendered||`, quaternion angular error `2 acos(clamp(|dot(q_gt,q_rendered)|,0,1))`
- 요약: p50/p95/p99와 평균, 표준편차; motion stability는 frame 간 velocity/acceleration variation 등 재현 가능한 식을 먼저 정의

Immediate와 buffered interpolation을 동일 trace/seed에서 비교합니다. buffer 증가로 얻는 error·underrun·motion stability 개선과 추가 display latency 비용을 함께 제시하며 예상과 다른 결과도 그대로 기록합니다. 산출물은 raw CSV/JSON, graph, 분석과 limitation입니다.

## 플랫폼별 impairment

Linux에서는 예를 들어 loopback 실험에 `tc qdisc ... netem delay 50ms 20ms loss 1% reorder 1%`를 적용할 수 있으나 정확한 interface와 기존 qdisc를 먼저 확인하고 종료 시 복구해야 합니다. Windows proxy와 Linux 적용/복구 도구, CSV 기록 및 experiment orchestrator는 모두 직접 구현할 범위입니다.

이 문서는 완료 결과가 아니라 실험 계약입니다. 최종 buffer 정책은 구현 후 측정 결과로 결정합니다.
