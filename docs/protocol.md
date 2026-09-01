# UDP 바이너리 프로토콜

Pose는 과거 값의 완전한 전달보다 최신 상태의 빠른 반영이 중요하므로 UDP를 사용합니다. 손실된 pose의 재전송 때문에 새 pose 처리를 막지 않고, application이 loss/reorder/stale 정책을 결정합니다. 즉 “UDP가 단순히 빠르기 때문”이 아니라 freshness를 reliability보다 우선한 선택입니다.

## Version 1 packet (64 bytes)

| Offset | Size | Field | Encoding |
|---:|---:|---|---|
| 0 | 4 | magic `PLNK` | `0x504C4E4B`, big-endian |
| 4 | 1 | version | `1` |
| 5 | 1 | message type | pose = `1` |
| 6 | 1 | valid | `0` or `1` |
| 7 | 1 | reserved | `0` |
| 8 | 4 | sequence | uint32, big-endian |
| 12 | 8 | sender timestamp | uint64 microseconds, big-endian |
| 20 | 4 | object ID | uint32, big-endian |
| 24 | 12 | position x/y/z | IEEE-754 float32 bits, big-endian |
| 36 | 16 | quaternion x/y/z/w | IEEE-754 float32 bits, big-endian |
| 52 | 12 | reserved | all zero |

논리 `PoseSample`의 메모리를 그대로 `send()`하지 않습니다. C++ struct padding/alignment, host endianness, ABI와 compiler 차이, protocol version 확장을 피하기 위해 field별로 직렬화합니다. 실제 제품에서는 Protobuf나 FlatBuffers도 대안입니다.

구현할 Decoder는 정확히 64 bytes인지, magic/version/type과 reserved/valid 값이 맞는지, position/quaternion이 finite인지, quaternion norm이 허용 오차 안인지 검사해야 합니다. timestamp/object ID의 의미 범위와 checksum/authentication은 초기 필수 범위에서 제외합니다.

## Sequence와 timestamp

- Sequence는 도착 순서를 분석하여 gap(loss 추정), duplicate, reorder를 탐지합니다.
- Timestamp는 pose의 생성 시점이며 buffer 정렬과 render sample 선택에 사용합니다.

Analyzer는 단순히 가장 최근 도착 packet과만 비교하지 않습니다. 그렇게 하면 reorder 뒤 정상 packet에서 loss를 과대계상할 수 있습니다. highest-seen sequence와 제한된 receive window를 사용해 wrap-around, late reorder와 duplicate를 구분하도록 구현합니다.

동일 머신의 두 process에서는 `std::chrono::steady_clock` epoch를 공유한다고 가정해 packet age와 display latency를 계산할 수 있습니다. 서로 다른 PC의 steady clock 값은 직접 비교할 수 없으므로 clock synchronization 또는 offset estimation 없이는 sender timestamp로 end-to-end latency를 주장하지 않습니다.
