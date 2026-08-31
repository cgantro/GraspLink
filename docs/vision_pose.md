# Vision Pose

Calibration is a separate ChArUco workflow that writes camera intrinsics and distortion coefficients to YAML. Runtime uses one known-size ArUco marker and `SOLVEPNP_IPPE_SQUARE`; marker size is supplied in metres. `SyntheticPoseSource` is the default deterministic source for network and interpolation experiments. The current OpenCV adapter is deliberately gated on an installed OpenCV build.
