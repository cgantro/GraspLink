# PoseLink

PoseLink streams timestamped 6DoF poses from a Vision Node to an OpenGL Viewer over UDP. CMake produces one user-facing executable, `poselink`, which is started in separate process modes: `poselink vision --port 5000`, `poselink viewer --port 5000`, and, for Windows impairment experiments, `poselink proxy --listen 5001 --target-port 5000 --delay-ms 50 --jitter-ms 20 --loss 1`.

See `docs/` for the protocol, coordinate contract, vision workflow, platforms, and repeatable experiments.
