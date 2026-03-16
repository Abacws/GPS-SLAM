# GPS-SLAM — Code Overview & Insights

> "GPS" = **G**aussian-**P**lus-**S**DF. Not satellite GPS. Not IMU. Just RGB-D.

---

## What It Is

A real-time **online 3D reconstruction** system that fuses two representations:

- **TSDF (via InfiniTAM)** — robust voxel-based geometry + pose tracking (KinectFusion-style)
- **3D Gaussian Splatting** — high-fidelity appearance, initialized and guided by the TSDF

The key insight: let the SDF handle geometry and tracking (fast, robust), and use far fewer Gaussians only for appearance refinement on top of it. Result: 150+ FPS on an RTX 4090.

---

## Online or Offline?

**Online.** Processes frames sequentially as they stream in:

1. Each frame → TSDF fusion (InfiniTAM)
2. Every `local_opt_interval` frames → raycast TSDF → initialize new Gaussians → local optimization window → prune Gaussians
3. Keyframes selected by rotation/translation thresholds

No global bundle adjustment. No loop closure. Purely incremental.

---

## Camera Pose: Given or Estimated?

**Both modes are supported**, controlled by `use_gt_pose` in the YAML config:

| `use_gt_pose` | Behavior |
|---|---|
| `false` (default) | InfiniTAM estimates poses via ICP on the TSDF (online tracking) |
| `true` | Ground-truth poses injected directly; tracking is disabled (used for benchmarking) |

GPS-SLAM itself does **not** implement any tracking. It fully delegates to InfiniTAM and reads back the `c2w` (camera-to-world) transform each frame via `GetTrackingState()->pose_d`.

---

## Inputs Required

| Input | Notes |
|---|---|
| RGB frames | Standard color images |
| Depth maps | Dense depth from RGB-D camera |
| Camera intrinsics | `fx, fy, cx, cy` — set in YAML config |
| Initial point cloud | SfM-derived `.ply`, ~500k points, seeds Gaussians at startup |
| Camera poses | Only needed if `use_gt_pose: true` |

No GPS, no IMU, no LiDAR.

---

## Key Files

| File | Role |
|---|---|
| `slam_trainer.cpp` | Entry point — parses config, starts pipeline |
| `slam/slam_pipeline.cpp` | Main per-frame SLAM loop |
| `slam/InfiniTAM_tools.cpp` | InfiniTAM bridge: TSDF fusion + ICP tracking |
| `slam/slam_gs_model.h` | Gaussian model extended for SLAM |
| `src/raw_gs_model.cpp` | Gaussian forward/backward, densification, pruning |
| `src/dataset_reader.cpp` | Loads RGB-D frames, poses, intrinsics, point clouds |
| `gsplat/gsplat_wapper.cpp` | Bindings to CUDA Gaussian rasterizer |
| `include/tensor_math.h` | Pose math utilities |
| `remote_viewer.cpp` | TCP server for live Gaussian visualization |
| `configs/release/` | YAML configs per dataset/scene |

---

## Algorithm Pipeline (Per Frame)

```
RGB-D Frame
    │
    ▼
InfiniTAM TSDF Fusion
    ├── ICP tracking → estimated c2w pose
    └── Voxel update

    (every local_opt_interval frames)
    │
    ▼
Raycast TSDF → depth + color maps from current viewpoints
    │
    ▼
Initialize new Gaussians from raycast output
    │
    ▼
Local Gaussian optimization (Adam, sliding window)
    ├── L1/photometric loss on rendered vs GT RGB
    └── Densification + pruning
```

---

## Rendering Modes

- `train_mode: ges` — hybrid: TSDF provides base depth/color, Gaussians refine details
- `train_mode: raw` — pure Gaussian splatting (no SDF guidance)

---

## Outputs

| Output | Description |
|---|---|
| `gs_model/` | Trained Gaussian parameters |
| `tsdf_mesh.ply` | Mesh extracted from TSDF |
| `tsdf_engine/` | Serialized TSDF voxel grid |
| `val/render/` | Rendered images for evaluation |
| `val/pose/` | Estimated camera trajectory |

Metrics: PSNR, SSIM, LPIPS (image quality) — ATE (pose accuracy) — mesh accuracy.

---

## Build

```bash
# Prerequisites: CUDA 12.8, LibTorch, OpenCV
# Extract ThirdLibs.zip to project root first

bash build_third_libs.sh
mkdir build && cd build && cmake .. && make -j$(nproc)
```

```bash
./build/slam_trainer configs/release/replica/office0.yaml
./build/remote_viewer configs/viewer/office0.yaml   # for live visualization
```

---

## Limitations / Notes

- Single-threaded currently (TSDF FPS and Gaussian FPS are sequential, not parallel)
- No loop closure or global map correction
- Tested only on Azure Kinect RGB-D; assumes dense depth
- Requires an initial SfM point cloud (not generated on-the-fly)
- Remote viewer is a separate client binary (Windows pre-built available)
