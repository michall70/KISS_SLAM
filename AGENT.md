# AGENTS.md

Goal: hand-roll ("手搓") a from-scratch reimplementation of KISS-SLAM / KISS-ICP in this repo, as a learning exercise. The installed packages are the reference — read them, don't edit them.

## Current status (2026-08)

手搓 KISS-ICP 里程计**已跑通**（KITTI 数据效果好、速度快）。学习者的代码全在 `scripts/`，可正常运行时。

已完成: preprocess → voxelize → adaptive threshold → vectorized point-to-plane ICP → VoxelHashMap 局部地图 → `register_frame` 完整里程计。
尚未开始: KISS-SLAM 层（局部地图切分 / 回环检测 / 位姿图优化）。

## Repo structure

- `scripts/` — 手搓代码（主战场）
- `AGENT.md`、`LEARNING_ROADMAP.md` — 学习计划与备忘
- 根目录输出: `kitti_output/`、`mydata_output/`、`results/`（已 gitignore）

## Hand-rolled modules (`scripts/`)

| 文件 | 内容 |
|---|---|
| `run_odometry.py` | **运行入口**。先 prompt 配置文件(轻量 import)→ 再加载 open3d → 跑里程计。运行: `cd scripts && python run_odometry.py` |
| `odometry.py` | `Odemetry` 类: `register_frame` 组装所有组件（纯类模块，无 main） |
| `functions.py` | 公共函数库(开头有 docstring 列清单): `transform_points`、`exp_map`、`read_kitti_bin`、`read_npy`、`read_point_cloud`(自动判断 .bin/.npy)、`to_pcd`、`voxel_down_sample` 等 |
| `icp_synthetic.py` | `Kiss_ICP`: **向量化** point-to-plane ICP(scipy cKDTree 批量查最近邻 + numpy 矩阵运算) + Geman-McClure 鲁棒核 |
| `VoxelHashMap.py` | `VoxelHashMap`: dict 实现(体素索引→点)，每体素 1 点，`update` 移除远点 |
| `threshold.py` | `AdaptiveThreshold`: EMA 自适应阈值 |
| `preprocessor.py` | `Preprocessor`: 距离裁剪 + deskew; `get_timestamps`: 从 start/end 时间戳文件构造逐点时间戳 |
| `config_loader.py` | 加载 yaml 配置(自动把字符串 `"None"` 转成 None) |
| `*.yaml` | 配置文件(见下) |
| `visualize.py`、`print_log.py` | 可视化/日志工具 |
| `exp_map.py`、`voxel_down_sample.py`、`transform.py` | 早期单文件练习(已被 functions.py 取代，历史遗留) |

## Config system

- yaml 配置文件按官方分类: `data` / `mapping` / `registration` / `adaptive_threshold`。
- 现有: `kitti_config.yaml`(KITTI 数据)、`mydata_config.yaml`(npy 数据)。**没有默认 `config.yaml`** → 运行时"直接回车"会失败，必须输入文件名。
- `run_odometry.py` 运行时列出可用 yaml 并询问文件名。输入会去引号、支持相对(相对 scripts/)和绝对路径。
- **坑**: yaml 里 `None` 会被解析成字符串 `"None"`(非标准空值)，`config_loader._normalize` 已兼容; 更规范写法是 `null`。

## Environment

- Use the `pyslam` conda env: `/home/michall/anaconda3/envs/pyslam/bin/python` (or `conda activate pyslam`).
- The system `python` does NOT have these packages — always point at the env binary.
- Versions: open3d 0.19.0, numpy 1.26.4, scipy 1.17.1, pydantic 2.13.4.
- **open3d import 约 3.7 秒**(加载大量 C++ 绑定)，不是 bug。`run_odometry.py` 把 prompt 放在 open3d 加载之前，避免启动干等。

## Data

- KITTI raw LiDAR (154 帧, drive_0005_sync), `.bin`(每点 4×float32 x,y,z,intensity):
  `/media/michall/学习资料/Michall/datasets/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data`
  + 同目录 `timestamps_start.txt` / `timestamps_end.txt`(帧级时间戳，无逐点时间戳→deskew 跳过)。
- 第二套 npy 数据 (`(N,3)` 数组): `/media/michall/学习资料/Michall/datasets/2D_to_depth/seq0/pointcloud`。
- `read_point_cloud` 自动按扩展名读 `.bin` / `.npy`。

## Reference implementation (read-only)

- `kiss_icp` v1.3.0 — LiDAR odometry. `<env>/lib/python3.11/site-packages/kiss_icp/`
- `kiss_slam` v0.0.2 — SLAM on top of kiss-icp. `.../kiss_slam/`
- `map_closures` v2.1.0 — loop-closure detection. `.../map_closures/`

## Critical architecture fact

The Python code is a thin glue layer. ALL heavy math is C++ exposed via pybind11 `.so` files, and must be reimplemented in numpy/open3d for the hand-rolled version:

- `kiss_icp.pybind.kiss_icp_pybind`: `_Registration` (point-to-plane ICP), `_VoxelHashMap`, `_AdaptiveThreshold`, `_Preprocessor` (motion-compensation/deskew), `_voxel_down_sample`.
- `kiss_slam.kiss_slam_pybind.kiss_slam_pybind`: `_VoxelMap`, `_PoseGraphOptimizer` (g2o), `_OccupancyMapper`.
- `map_closures.pybind.map_closures_pybind`: density-map + HBST descriptor loop-closure search.

Don't copy the `.py` wrappers and assume the algorithm is done — the real algorithm lives in the `.so`. 手搓时用 numpy/scipy/open3d 复现这些内核。

## Odometry data flow (reference `kiss_icp/kiss_icp.py:43`)

deskew → voxelize at `0.5*voxel_size`(建图) then `1.5*voxel_size`(配准) → adaptive threshold `sigma` → ICP initial guess `last_pose @ last_delta` → point-to-plane ICP with `max_corr=3*sigma`, `kernel=sigma` → `model_deviation` 更新阈值 → `local_map.update` → `last_delta = inv(last_pose)@new_pose; last_pose = new_pose`(先 delta 后 pose)。

SLAM 层(KISS-SLAM): `KissSLAM.process_scan`(`kiss_slam/slam.py:61`)、`generate_new_node`(`slam.py:86`)、`fine_grained_optimization`(`slam.py:117`) — 尚未手搓。

## Hand-rolled conventions & gotchas (踩过的坑)

- 变换约定(行向量右乘): `transform_points(pts, T) = pts @ R.T + t`。所有位姿合成用 `keypose @ rel_pose`。
- `exp_map(xi)`: `xi = [rho(前3), phi(后3)]`，罗德里格斯公式。
- **deskew 方向**: 对齐到扫描结束时刻 s=1，s=0 的点应用 `inv(relative_motion)` 的完全变换(不是 relative_motion 本身！)。中间点按 `(1-s)` 比例插值旋转角和平移。
- **voxel_down_sample 返回"每个体素的第一个点"，不是质心**(实验验证 KISS 的行为)。
- **VoxelHashMap 每体素 1 点(第一个)**，`max_points_per_voxel` 参数对去重无影响(历史遗留)。`add_points` 不移远点，`update` 才移除(以 pose 平移为中心)。
- **AdaptiveThreshold**: `deviation = max(rot_angle × max_range, trans_norm)`(旋转用 max_range 换算成米)，EMA 平滑 alpha≈0.4，下限 `min_motion_th`。
- **鲁棒核必须有**: 无核的高斯牛顿会被离群点平方加权带偏("initial_guess 好但 ICP 后更坏")。用 Geman-McClure `w = (σ²/(σ²+r²))²`。
- **向量化提速**: scipy `cKDTree.query(pts, k=1)` 批量查最近邻 + numpy 矩阵运算(`H=(w[:,None]*J).T@J`)替代逐点 Python 循环(快 25-75 倍)。
- 相对路径依赖工作目录: `config_loader` 用 `Path(__file__).parent` 定位，从任何目录运行都行。
- KITTI `.bin` 无逐点时间戳 → deskew 实际跳过(官方 `ReadKITTI` 返回空 timestamps)。

## Reference conventions & gotchas

- World pose of a node's frame `i`: `node.keypose @ node.local_trajectory[i]`.
- Traveled-distance check uses `norm(current_pose[:3,-1])` (distance from origin), not accumulated (`kiss_slam/slam.py:67`).
- Defaults: `mapping.voxel_size = max_range/100` (default 100 → 1.0 m); local mapper `voxel_size=0.5`, `splitting_distance=100.0`; adaptive threshold `initial_threshold=2.0`, `min_motion_th=0.1`.
- Loop-closure acceptance: MapClosures candidate → ICP refine → overlap ratio `> 0.4`.
- Config is pydantic `BaseSettings` with `env_prefix="kiss_icp_"` / `"kiss_slam_"`.
- `voxel_down_sample` uses double (`_Vector3dVector`); `VoxelMap` uses float32 (`_Vector3fVector`).
- 已知诡异现象: KISS 的 `_VoxelHashMap` 体素边界不是严格 `floor(x/voxel_size)`(浮点/哈希细节，0.3 vs 1.0 现象)。手搓用 floor 即可，边界点差异不影响整体精度。

## Commands

- 手搓: `cd scripts && python run_odometry.py`(运行时会询问配置文件名)。
- 参考 CLI: `kiss_icp_pipeline`, `kiss_slam_pipeline`, `kiss_icp_dump_config`, `kiss_slam_dump_config`。
- 官方输出对比基准: `results/latest/data_poses.npy`（kiss_icp_pipeline 跑出来的轨迹）。
