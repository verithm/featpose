# featpose

Monocular 6-DoF pose estimation from sparse local features, as a ROS 2 package.
Built as an early perception experiment for the [22-DoF humanoid
project](https://github.com/RHplusLab) at Yonsei Robotics Club ROBOIN.

**This approach did not work well enough to use, and the project moved to
AprilTag and color-based detection instead.** The code is kept as a record of
what was tried and why it failed.

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

## Why it failed

The method assumes each descriptor sits reliably on a known vertex of a
textured, rigid object. The parts we actually needed to localize did not satisfy
that. They were largely untextured plastic and metal, so ORB found few stable
keypoints, and the ones it did find were dominated by specular highlights and
edges that shift with viewpoint. With too few correct correspondences,
`solvePnPRansac` either failed to find a consensus or locked onto a wrong pose
that looked plausible for a single frame and jumped between frames.

The deeper problem was compute. The alternative was a learned 6D pose estimator,
which we could not train or run at the time given the hardware available to the
project. Classical features were the cheap option; they were also the wrong one
for these objects.

Fiducial markers solved the problem instead. AprilTag supplies exactly the
texture that ORB was missing, in a form designed for reliable pose recovery — at
the cost of having to attach markers to the world.

---

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
