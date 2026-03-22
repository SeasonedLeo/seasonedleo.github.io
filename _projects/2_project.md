---
layout: page
title: Visual Odometry
description: RGB-D visual odometry pipeline with step-by-step implementation details
img: assets/img/3.jpg
importance: 2
category: work
giscus_comments: false
related_publications: false
---

## Overview

This project implements RGB-D visual odometry from a robot camera, grounded in a
clear camera projection model. A 3D world point $O_w = [x, y, z, 1]^T$ is first mapped
into the camera frame using the extrinsics $(R, t)$, then projected onto the image
plane with intrinsics $K$:

$$
O_c = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix} O_w,\quad
\tilde{o} = K \, [I \; 0] \, O_c,\quad
(u, v) = \left(\frac{\tilde{x}}{\tilde{z}}, \frac{\tilde{y}}{\tilde{z}}\right)
$$

This pinhole model lets us project world points into pixels and back-project pixels
into 3D using depth. The pipeline then tracks features across RGB frames, lifts them
to 3D, estimates camera motion with $R,t$, and integrates pose over time to recover
the trajectory. Along the way, we align image coordinates with pixel coordinates via
scaling and principal point offsets in $K$, consistent with how 2D images are stored
as $M \times N \times 3$ RGB arrays in software.

## Inputs And Assumptions

- Synchronized RGB and depth frames from the same camera.
- Known camera intrinsics `K` with `(fx, fy, cx, cy)`.
- Depth is in meters (or converted to meters), with invalid depth masked out.
- Consecutive frames have sufficient overlap and small inter-frame motion.

### What "Known Intrinsics K" Means

Camera intrinsics are the calibration parameters that map 3D camera coordinates
to 2D pixel coordinates. They are typically fixed for a given camera once
calibrated.
model the cameras projective geometry through the coordinate
system transformation. These transformations can
be used to project points from the world frame
to the image frame, using the pinhole camera model . 

The intrinsic matrix is:

```
K = [[fx,  0, cx],
     [ 0, fy, cy],
     [ 0,  0,  1]]
```

Where:

- `fx, fy`: focal lengths in pixels.
- `cx, cy`: principal point (optical center) in pixels.

You need `K` to back-project pixels to 3D and to project 3D points to pixels.

### Calibration Example (OpenCV)

This is a minimal example of how intrinsics are estimated using a checkerboard.
In practice, you collect 10–30 images of a checkerboard at different angles.

```python
import cv2
import numpy as np

# Checkerboard inner corners (rows, cols)
pattern_size = (9, 6)
square_size = 0.024  # meters

# 3D points in checkerboard coordinate system
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
objp *= square_size

objpoints = []  # 3D points
imgpoints = []  # 2D points

image_paths = ["img1.jpg", "img2.jpg", "img3.jpg"]  # replace with your files

for path in image_paths:
    img = cv2.imread(path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, pattern_size)
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("K:", K)
print("dist:", dist.ravel())
```

## Parameter Glossary

Camera and geometry:

- `K`: 3x3 intrinsic matrix. Converts normalized camera coordinates to pixels.
- `fx, fy`: focal lengths in pixels. Larger values mean narrower field of view.
- `cx, cy`: principal point in pixels (optical center).
- `u, v`: pixel coordinates (column, row).
- `Z`: depth value at `(u, v)` in meters.
- `P_c`: 3D point in the camera frame.

Pose and transforms:

- `R`: 3x3 rotation matrix. Maps points from previous camera frame to current.
- `t`: 3x1 translation vector in meters.
- `T`: 4x4 homogeneous transform `[R t; 0 1]`.
- `T_k-1,k`: relative pose from frame `k-1` to `k`.
- `T_k`: global pose at time `k` in world coordinates.

Optimization:

- `p_i`: observed 2D pixel in frame `k`.
- `P_i`: 3D point from frame `k-1` lifted using depth.
- `π(·)`: projection from 3D to pixel coordinates.
- `e_i`: reprojection error in pixels.
- `τ`: inlier threshold in pixels for RANSAC.

## Pipeline Steps

1. Load RGB and depth frames, and ensure time alignment.
2. Detect keypoints and descriptors in the current RGB frame.
3. Match descriptors to the previous frame and filter outliers.
4. For matched keypoints, lift 2D pixels to 3D using the previous frame depth.
5. Solve for relative pose `(R, t)` using PnP with RANSAC.
6. Refine pose with non-linear optimization on inliers (optional).
7. Compose the global pose and store the camera trajectory.
8. Repeat for the next frame.

## Core Math

- Back-project a pixel `(u, v)` with depth `Z` to 3D in camera coordinates:
  - `X = (u - cx) * Z / fx`
  - `Y = (v - cy) * Z / fy`
  - `Z = Z`

- Pose composition (world to camera):
  - `T_wc = T_wc * T_ccprev`

## Mathematics Of Visual Odometry

This section summarizes the geometry and optimization used in RGB-D VO.

### 1. Camera Model And Projection

We use the pinhole camera model:

- Intrinsics:
  - `K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]`
- 3D point in camera coordinates: `P_c = [X, Y, Z]^T`
- Projection to pixel:
  - `[u, v, 1]^T = K * [X/Z, Y/Z, 1]^T`

Python function:

```python
import numpy as np

def project_point(P_c, K):
    X, Y, Z = P_c
    if Z <= 0:
        return None
    x = X / Z
    y = Y / Z
    u = K[0, 0] * x + K[0, 2]
    v = K[1, 1] * y + K[1, 2]
    return np.array([u, v], dtype=np.float64)
```

### 2. Back-Projection With Depth

Given pixel `(u, v)` and depth `Z`:

- `X = (u - cx) * Z / fx`
- `Y = (v - cy) * Z / fy`
- `P_c = [X, Y, Z]^T`

Python function:

```python
import numpy as np

def backproject_pixel(u, v, z, K):
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    X = (u - cx) * z / fx
    Y = (v - cy) * z / fy
    return np.array([X, Y, z], dtype=np.float64)
```

### 3. Rigid Motion In SE(3)

Relative pose between consecutive frames is a rigid transform:

- `T = [R t; 0 1]` with `R ∈ SO(3)`, `t ∈ R^3`
- Point transformation:
  - `P_c2 = R * P_c1 + t`

Python function:

```python
import numpy as np

def transform_point(P_c1, R, t):
    return R @ P_c1 + t.reshape(3)
```

### 4. RGB-D Odometry As 3D-2D PnP

For matched features, we use depth from frame `k-1` to form 3D points, and
their matched pixel locations in frame `k` as 2D observations.

Let:

- `P_i` be 3D points from frame `k-1`
- `p_i` be 2D pixels in frame `k`

We solve:

- `p_i ≈ π( K * (R * P_i + t) )`

where `π([x, y, z]^T) = [x/z, y/z]^T`.

This is the standard Perspective-n-Point (PnP) problem.

Python function (build 3D-2D correspondences):

```python
import numpy as np

def build_3d_2d_correspondences(kp_prev, kp_curr, matches, depth_prev, K):
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    pts_3d = []
    pts_2d = []

    for m in matches:
        u_prev, v_prev = kp_prev[m.queryIdx].pt
        u_curr, v_curr = kp_curr[m.trainIdx].pt

        z = depth_prev[int(round(v_prev)), int(round(u_prev))]
        if z <= 0 or np.isnan(z):
            continue

        X = (u_prev - cx) * z / fx
        Y = (v_prev - cy) * z / fy
        pts_3d.append([X, Y, z])
        pts_2d.append([u_curr, v_curr])

    if len(pts_3d) == 0:
        return None, None

    return np.array(pts_3d), np.array(pts_2d)
```

### 5. Reprojection Error Objective

The pose `(R, t)` is estimated by minimizing reprojection error:

- `min_{R,t} Σ || p_i - π( K * (R * P_i + t) ) ||^2`

Where:

- `p_i` is the measured pixel in frame `k`.
- `π(·)` divides by depth to map to pixels.
- The error is measured in pixel units.

Python function (PnP with RANSAC):

```python
import cv2

def estimate_pose_pnp(pts_3d, pts_2d, K):
    if pts_3d is None or len(pts_3d) < 4:
        return None, None, None

    success, rvec, tvec, inliers = cv2.solvePnPRansac(
        pts_3d,
        pts_2d,
        K,
        None,
        iterationsCount=200,
        reprojectionError=3.0,
        confidence=0.999,
    )
    if not success or inliers is None or len(inliers) < 6:
        return None, None, None

    return rvec, tvec, inliers
```

In practice:

- Use `solvePnPRansac` to reject outliers.
- Optionally refine with `solvePnP` or `cv2.solvePnPRefineLM`.

### 6. RANSAC Inlier Test

For each hypothesis pose:

- Project `P_i` into frame `k`.
- Compute reprojection error `e_i`.
- Mark as inlier if `e_i < τ` (typically 2–5 px).

The final pose is re-estimated using only inliers.

Typical parameter meanings:

- `τ`: maximum reprojection error to accept an inlier, in pixels.
- `iterations`: number of random pose hypotheses tried.
- `confidence`: probability that at least one hypothesis is outlier-free.

Python function (compute reprojection errors):

```python
import numpy as np

def reprojection_errors(pts_3d, pts_2d, rvec, tvec, K):
    R, _ = cv2.Rodrigues(rvec)
    errors = []
    for P, p in zip(pts_3d, pts_2d):
        P_c = R @ P + tvec.reshape(3)
        proj = project_point(P_c, K)
        if proj is None:
            errors.append(np.inf)
        else:
            errors.append(np.linalg.norm(p - proj))
    return np.array(errors)
```

### 7. Pose Composition Over Time

If `T_k` is the global pose at time `k`, and `T_k-1,k` is the relative pose:

- `T_k = T_k-1 * T_k-1,k`

Python function:

```python
import numpy as np
import cv2

def compose_pose(T_wc, rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T_wc @ T
```

This integrates the trajectory in the world frame.

### 8. Optional Bundle Adjustment (BA)

For higher accuracy, optimize poses and points jointly over a window:

- `min_{R_k,t_k,P_i} Σ_k Σ_i || p_i^k - π( K * (R_k * P_i + t_k) ) ||^2`

This improves consistency but is more expensive.

### 9. Common Failure Modes (Math Perspective)

- Pure rotation or low parallax: poor translation observability.
- Noisy depth: back-projected 3D points become unreliable.
- Repetitive textures: incorrect correspondences increase outliers.

## Python Implementation

Below is a minimal, working reference implementation. It is structured so you can
drop it into a notebook or a script and extend it.

```python
import cv2
import numpy as np

# -------------------------
# Utilities
# -------------------------

def project_point(P_c, K):
    X, Y, Z = P_c
    if Z <= 0:
        return None
    x = X / Z
    y = Y / Z
    u = K[0, 0] * x + K[0, 2]
    v = K[1, 1] * y + K[1, 2]
    return np.array([u, v], dtype=np.float64)


def backproject_pixel(u, v, z, K):
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    X = (u - cx) * z / fx
    Y = (v - cy) * z / fy
    return np.array([X, Y, z], dtype=np.float64)


def transform_point(P_c1, R, t):
    return R @ P_c1 + t.reshape(3)


def extract_features(img_gray, n_features=2000):
    orb = cv2.ORB_create(nfeatures=n_features)
    keypoints, descriptors = orb.detectAndCompute(img_gray, None)
    return keypoints, descriptors


def match_features(desc1, desc2, ratio=0.75):
    if desc1 is None or desc2 is None:
        return []
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    raw = bf.knnMatch(desc1, desc2, k=2)
    good = []
    for m, n in raw:
        if m.distance < ratio * n.distance:
            good.append(m)
    return good


def build_3d_2d_correspondences(kp_prev, kp_curr, matches, depth_prev, K):
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    pts_3d = []
    pts_2d = []

    for m in matches:
        u_prev, v_prev = kp_prev[m.queryIdx].pt
        u_curr, v_curr = kp_curr[m.trainIdx].pt

        z = depth_prev[int(round(v_prev)), int(round(u_prev))]
        if z <= 0 or np.isnan(z):
            continue

        p3d = backproject_pixel(u_prev, v_prev, z, K)
        pts_3d.append(p3d)
        pts_2d.append([u_curr, v_curr])

    if len(pts_3d) == 0:
        return None, None

    return np.array(pts_3d), np.array(pts_2d)


def estimate_pose_pnp(pts_3d, pts_2d, K):
    # Requires >= 4 points
    if pts_3d is None or len(pts_3d) < 4:
        return None, None, None

    success, rvec, tvec, inliers = cv2.solvePnPRansac(
        pts_3d,
        pts_2d,
        K,
        None,
        iterationsCount=200,
        reprojectionError=3.0,
        confidence=0.999,
    )
    if not success or inliers is None or len(inliers) < 6:
        return None, None, None

    return rvec, tvec, inliers


def compose_pose(T_wc, rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T_wc @ T


def reprojection_errors(pts_3d, pts_2d, rvec, tvec, K):
    R, _ = cv2.Rodrigues(rvec)
    errors = []
    for P, p in zip(pts_3d, pts_2d):
        P_c = transform_point(P, R, tvec)
        proj = project_point(P_c, K)
        if proj is None:
            errors.append(np.inf)
        else:
            errors.append(np.linalg.norm(p - proj))
    return np.array(errors)


# -------------------------
# Main Loop
# -------------------------

def run_visual_odometry(rgb_frames, depth_frames, K):
    assert len(rgb_frames) == len(depth_frames)

    T_wc = np.eye(4)
    trajectory = [T_wc.copy()]

    prev_gray = None
    prev_kp = None
    prev_desc = None
    prev_depth = None

    for idx, (rgb, depth) in enumerate(zip(rgb_frames, depth_frames)):
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

        kp, desc = extract_features(gray)

        if prev_gray is not None:
            matches = match_features(prev_desc, desc)
            pts_3d, pts_2d = build_3d_2d_correspondences(
                prev_kp, kp, matches, prev_depth, K
            )

            rvec, tvec, inliers = estimate_pose_pnp(pts_3d, pts_2d, K)
            if rvec is not None:
                T_wc = compose_pose(T_wc, rvec, tvec)
                trajectory.append(T_wc.copy())
            else:
                trajectory.append(T_wc.copy())
        else:
            trajectory.append(T_wc.copy())

        prev_gray = gray
        prev_kp = kp
        prev_desc = desc
        prev_depth = depth

    return trajectory
```

## Practical Notes

- Depth alignment matters. If the depth camera is offset from RGB, perform depth
  registration before back-projection.
- Filter out keypoints on invalid or very noisy depth values.
- If matches are weak, increase keypoints or switch to SIFT/AKAZE.
- For smoother trajectories, add a pose-graph or bundle adjustment stage.

## Expected Outputs

- A list of 4x4 camera poses `T_wc` over time.
- A trajectory plot (e.g., top-down `x-z` view) for qualitative evaluation.
