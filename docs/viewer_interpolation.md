# Viewer and Interpolation

Immediate mode renders the latest valid pose. Buffered mode renders at `now - bufferDelay`, uses LERP for position and SLERP for rotation, and retains only bounded timestamp-ordered history. With no bracketing pair it holds the nearest pose; stale/lost policy is driven by pose age. Viewer title metrics expose receive/loss/jitter and buffer delay.
