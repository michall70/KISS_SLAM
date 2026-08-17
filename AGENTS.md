# AGENTS.md

Goal: hand-roll ("手搓") a from-scratch reimplementation of KISS-SLAM / KISS-ICP in this repo, as a learning exercise. The installed packages are the reference — read them, don't edit them.

## Environment

- Use the `pyslam` conda env: `/home/michall/anaconda3/envs/pyslam/bin/python` (or `conda activate pyslam`).
- The system `python` does NOT have these packages installed — always point at the env binary.
- Versions: open3d 0.19.0, numpy 1.26.4, pydantic 2.13.4.

## Data

- KITTI raw (odometry-style) LiDAR data, short sequence `2011_09_26_drive_0005_sync`, 154 frames:
  `/media/michall/学习资料/Michall/datasets/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data`
- Format: KITTI `.bin`, each point 4×float32 `(x, y, z, intensity)` → read with `np.fromfile(f, np.float32).reshape(-1, 4)[:, :3]`.
- Readable directly by the `generic` dataloader (matches its `.bin` reader, `kiss_icp/datasets/generic.py`).

## Reference implementation (read-only)

- `kiss_icp` v1.3.0 — LiDAR odometry. `<env>/lib/python3.11/site-packages/kiss_icp/`
- `kiss_slam` v0.0.2 — SLAM on top of kiss-icp. `.../kiss_slam/`
- `map_closures` v2.1.0 — loop-closure detection. `.../map_closures/`

## Critical architecture fact

The Python code is a thin glue layer. ALL heavy math is C++ exposed via pybind11 `.so` files, and must be reimplemented in numpy/open3d for the hand-rolled version:

- `kiss_icp.pybind.kiss_icp_pybind`: `_Registration` (point-to-plane ICP), `_VoxelHashMap`, `_AdaptiveThreshold`, `_Preprocessor` (motion-compensation/deskew), `_voxel_down_sample`.
- `kiss_slam.kiss_slam_pybind.kiss_slam_pybind`: `_VoxelMap`, `_PoseGraphOptimizer` (g2o), `_OccupancyMapper`.
- `map_closures.pybind.map_closures_pybind`: density-map + HBST descriptor loop-closure search.

Don't copy the `.py` wrappers and assume the algorithm is done — the real algorithm lives in the `.so`.

## Pipeline / data flow

- Odometry `KissICP.register_frame` (`kiss_icp/kiss_icp.py:43`): deskew → voxelize at `0.5*voxel_size` then `1.5*voxel_size` → adaptive threshold `sigma` → ICP initial guess `last_pose @ last_delta` → point-to-plane ICP with `max_corr=3*sigma`, `kernel=sigma` → update map/delta/last_pose.
- SLAM `KissSLAM.process_scan` (`kiss_slam/slam.py:61`): register frame → downsample to `local_mapper.voxel_size` → integrate into `VoxelMap` → append pose to `local_trajectory` → when traveled distance > `splitting_distance`, call `generate_new_node`.
- `generate_new_node` (`kiss_slam/slam.py:86`): snapshot local map, reset odometry (transform map by inverse of last relative motion, `last_pose = eye(4)`), finalize a `LocalMapGraph` node, add pose-graph variable + odometry factor, then run loop closure.
- Pose-graph `fine_grained_optimization` (`kiss_slam/slam.py:117`): dense odometry factors per frame, first variable fixed, `np.eye(6)` information matrices, g2o written via `write_graph`.

## Conventions & gotchas

- Transform convention: `transform_points(pcd, T) = pcd @ R.T + t` — row vectors, right-multiply (`kiss_slam/slam.py:34`).
- World pose of a node's frame `i`: `node.keypose @ node.local_trajectory[i]` — `keypose` is the local-map origin in world frame (`LocalMap.endpose`, `kiss_slam/local_map_graph.py:38`).
- The traveled-distance check at `kiss_slam/slam.py:67` uses `norm(current_pose[:3,-1])` (distance from origin), not accumulated distance — replicate as-is to match behavior.
- Defaults that are easy to miss: `mapping.voxel_size = max_range/100` (default max_range=100 → 1.0 m); local mapper `voxel_size=0.5`, `splitting_distance=100.0`; adaptive threshold `initial_threshold=2.0`, `min_motion_th=0.1` (`kiss_icp/config/config.py`, `kiss_slam/config/config.py`).
- Loop-closure acceptance (`kiss_slam/loop_closer.py:44`): MapClosures candidate → ICP point-to-plane refine → overlap ratio `intersection / min(size_src, size_tgt) > 0.4`.
- Config is pydantic `BaseSettings` with `env_prefix="kiss_icp_"` / `"kiss_slam_"` — env vars override YAML defaults.
- `voxel_down_sample` uses double-precision (`_Vector3dVector`); `VoxelMap` uses float32 (`_Vector3fVector`).

## Commands (reference CLI)

- `kiss_icp_pipeline`, `kiss_slam_pipeline`, `kiss_icp_dump_config`, `kiss_slam_dump_config` (console scripts in `.../kiss_*dist-info/entry_points.txt`).
- `kiss_slam_pipeline <data-dir> --dataloader generic -n 100 --visualize`
