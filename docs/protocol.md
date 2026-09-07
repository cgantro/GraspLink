# UDP Object Pose Protocol

## 1. 목적

이 문서는 로드맵 2단계인 **UDP Object Pose**에서 사용할 wire protocol의 설계 기준을 정리한다.

현재 `Protocol.h/.cpp`, `UdpSocket.h/.cpp`는 master 기준으로 비어 있으므로 이 문서는 **구현 전 설계 문서**다.

---

## 2. 전송 대상

초기 전송 대상은 robot joint state가 아니라 **tracked object의 6DoF Pose**다.

```text
Vision/Synthetic Source
→ Object Pose
→ Encode
→ UDP
→ Decode
→ Simulation Object
```

Robot IK/FK는 Viewer/Simulator에서 수행한다.

---

## 3. 요구 field

최소 field 후보:

| Field | 목적 | 상태 |
|---|---|---|
| magic | packet 식별 | 필요 |
| version | protocol 확장 | 필요 |
| message type | 향후 message 구분 | 필요 |
| sequence | loss/reorder 분석 | 단계 2부터 넣는 것을 권장, 최종 확정 필요 |
| timestamp | sample 생성 시점 | 단계 2부터 넣는 것을 권장, 최종 확정 필요 |
| object ID | multi-object 확장 | 초기 포함 여부 결정 필요 |
| position x/y/z | object 위치 | 필요 |
| quaternion | object 방향 | 필요 |
| valid/status | detection 상태 전달 | ArUco 단계 전에 확정 |

---

## 4. Quaternion 순서

현재 C++ domain type `PoseLink::Quaternion`은:

```text
(w, x, y, z)
```

field를 가진다.

Wire format은 encode/decode가 명시적이면 다른 순서도 가능하지만 혼동 비용을 줄이기 위해 **wire도 `(w,x,y,z)`로 맞추는 안을 우선 검토**한다.

최종 byte layout을 구현하기 전에 이 결정을 확정한다.

---

## 5. Serialization 원칙

하지 않는 것:

```cpp
sendto(socket, &pose, sizeof(pose), ...);
```

이유:

- C++ struct padding
- alignment
- ABI 차이
- host endianness
- version 확장 어려움

대신 field별로 byte buffer에 encode한다.

```text
Logical Pose
→ Encoder
→ byte buffer
→ UDP
```

Decoder는 역순으로 각 field를 검증한다.

---

## 6. Endianness

정수 field는 network byte order(big-endian)를 기본 후보로 한다.

float는 IEEE-754 bit pattern을 고정 크기 정수처럼 변환하여 명시적으로 byte order를 맞추는 방식이 가능하다.

구현 시 다음을 unit test로 고정한다.

```text
known values
→ expected bytes
→ decode
→ original values
```

---

## 7. Packet Validation

Decoder는 최소 다음을 확인한다.

- expected packet size
- magic
- supported version
- supported message type
- finite position value
- finite quaternion value
- quaternion norm이 유효한 범위인지

invalid packet은 Viewer state를 직접 수정하지 않는다.

---

## 8. Sequence

Sequence Number는 마지막 network experiment만을 위한 값이 아니라 receiver correctness 검증에도 유용하다.

예:

```text
1, 2, 4, 3, 4
```

를 받아:

- gap
- late reorder
- duplicate

를 구분할 수 있어야 한다.

단순히 직전 도착 sequence와만 비교하면 reorder 상황에서 잘못된 loss 계산이 가능하므로 highest-seen/window 방식 등을 구현 단계에서 결정한다.

---

## 9. Timestamp

Timestamp는 Pose가 생성된 시점을 표현한다.

용도:

- packet age
- arrival interval 분석
- 이후 PoseBuffer/interpolation
- network jitter 실험

같은 머신의 두 process에서는 monotonic clock 비교가 가능하지만, 서로 다른 PC의 `steady_clock` epoch는 직접 비교할 수 없다.

원격 PC 간 정확한 E2E latency를 측정하려면 clock synchronization 또는 offset estimation이 필요하다.

---

## 10. 단계 2 완료 기준

`UDP Object Pose` 단계에서는 최소 다음을 검증한다.

```text
Synthetic Pose
→ Encode
→ UDP loopback
→ Decode
→ Viewer Transform
```

성공 조건:

- local direct Pose와 UDP round-trip Pose가 허용 오차 내에서 일치
- malformed packet이 crash를 유발하지 않음
- process를 분리해도 Cube/object motion이 동일하게 재현됨
- packet count/sequence를 확인할 수 있음

정량 성능 목표는 baseline 측정 전에는 확정하지 않는다.

---

## 11. 구현 전에 결정할 항목

1. exact packet byte layout
2. packet 전체 크기
3. wire quaternion order
4. sequence 시작값/wrap 처리
5. timestamp unit
6. object ID 포함 여부
7. invalid detection 표현 방식
8. socket blocking/non-blocking 정책

이 결정이 끝나면 `Protocol.h/.cpp`, `UdpSocket.h/.cpp`를 구현한다.
