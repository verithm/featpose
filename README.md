# featpose

Monocular 6-DoF pose estimation from sparse local features, as a ROS 2 package.
Built as an early perception experiment for the [22-DoF humanoid
project](https://github.com/RHplusLab) at Yonsei Robotics Club ROBOIN.

**This approach did not work well enough to use, and the project moved to
AprilTag and color-based detection instead.** The code is kept as a record of
what was tried.

---

## How it works

An object is represented offline as a `.npz` model holding ORB descriptors, the
3D vertices of the object, and a mapping from each descriptor to the vertex it
sits on. At runtime:

1. ORB keypoints and descriptors are extracted from the camera frame.
2. They are matched against the model descriptors, with either a brute-force
   matcher (top 10 by distance) or FLANN with Lowe's ratio test at 0.75.
3. Matched descriptors are looked up through `descriptors_to_vertices` to recover
   3D–2D correspondences, and `solvePnPRansac` gives rotation and translation.
4. The pose is published as a `PoseStamped` on `/featpose/pose`, and an
   annotated frame with the projected wireframe on `/featpose/image`.

A cube model is included for testing.

---

## Why it didn't work out

The failure was in the descriptor, not the pose solver. ORB is built on FAST
corners and a binary descriptor, so it needs surfaces with enough texture to
produce corners that reappear in the same place across viewpoints. Robot parts
mostly do not have that. Where ORB did find keypoints, they were too few and too
unstable for `solvePnPRansac` to settle on a consistent pose.

The offline model makes this worse rather than better. Tying each descriptor to a
specific vertex assumes descriptors land on vertices and stay there — an
assumption that only holds for objects whose texture is fixed to their geometry.

A learned 6D pose estimator was tried separately, outside this package. It ran
into a different wall: we could not secure the compute to train and run one
within the project.

Fiducial markers solved the problem instead. AprilTag supplies exactly the
texture and known geometry that feature matching was missing, in a form designed
for reliable pose recovery — at the cost of having to attach markers to the
world. That, with color-based detection, is what the humanoid ended up using.

## Layout

```
src/featpose/featpose.py           feature matching, PnP, publishing
src/featpose/featpose_webcam.py    webcam entry point
config/featpose_webcam.yaml        model type, matcher, camera intrinsics
launch/featpose_webcam.launch.py
model/cube.npz                     descriptors, vertices, edges for a test cube
rviz/featpose.rviz
```

## Running

```bash
colcon build --packages-select featpose
ros2 launch featpose featpose_webcam.launch.py
```

Set `camera_matrix` and `dist_coeffs` in `config/featpose_webcam.yaml` from your
own camera calibration — the defaults are for a 1920×1080 nominal camera and are
not calibrated for any real device.

---

Apache-2.0.
