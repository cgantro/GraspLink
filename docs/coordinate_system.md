# Coordinate System

All distances are metres and rotations are normalized `(x,y,z,w)` quaternions. OpenCV pose is marker/object relative to the camera (+X right, +Y down, +Z forward). The Viewer must apply one documented conversion before its OpenGL model/view/projection matrices; world axis, marker axis, camera-forward direction, and object trajectory use that same conversion.
