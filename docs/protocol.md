# Protocol

Pose packets are fixed 64-byte binary messages: magic `PLNK`, version, type, validity, sequence, steady-clock timestamp in microseconds, object ID, float32 position in metres, and float32 quaternion `(x,y,z,w)`. Integers are big-endian; floats use their IEEE-754 bit pattern. Decoder rejects wrong size, magic, version/type, non-finite values, and non-normalized quaternions.

Sequence is used for loss/reorder/duplicate detection; timestamps select buffer samples. Without clock synchronization, sender timestamps must not be used as cross-PC end-to-end latency.
