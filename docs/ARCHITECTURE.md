# Architecture

`vision_node` produces `PoseSample` values and serializes them through `pose_common`; `pose_viewer` validates, analyzes, buffers, and renders received poses. Capture/vision and sending are intended to be separated by a bounded DROP_OLDEST queue when the camera source is enabled. Shutdown order is source stop, queue close, worker join, socket close, then OpenGL destruction.

`network_proxy` is a separate process and never changes the Vision/Viewer data path. `pose_common` owns protocol, socket, time, sequence metrics, and pose interpolation.

`viewer_graphics` preserves the reusable MiniBCG OpenGL resource layer: shader compilation/uniform cache, VBO, VAO, and RGBA texture ownership. Broadcast-only Scene, media, and lower-third skeletons are intentionally excluded.
