# KISS-SLAM 手搓学习路线

> 面向：Python 新手 + SLAM 新手（仅《视觉 SLAM 十四讲》基础），目标是**从零用 numpy / open3d 手搓一个 KISS-SLAM**。
> 配套参考：本仓库 `AGENTS.md`（记录参考实现位置与架构，写代码前先读它）。

---

## 0. 总目标与核心策略

**目标**：不调用 kiss_icp / kiss_slam 的 C++ 内核，用纯 Python 复现一个能跑的 3D LiDAR SLAM 系统，包含：

1. LiDAR 里程计（点云预处理 → 体素化 → ICP 配准 → 局部地图）
2. 局部地图管理（子图切分）
3. 回环检测（描述子匹配 + ICP 验证）
4. 位姿图优化（g2o）

**核心策略（本项目最值钱的一条）**：参考实现已经装在环境里，且所有重活都在 C++ pybind11 的 `.so` 里，但 `.so` 的**函数可以直接在 Python 里调用**。所以每一步你都可以：

> 用你自己的 numpy 实现 vs. 官方 `.so` 函数，喂同样的输入，对比输出是否一致。

这把手搓从"黑盒照抄"变成了**单元级测试驱动学习**。每个模块都这样验证通过，最后拼起来的大系统就基本可信。详见第 5 节。

**总数据流**（写代码时反复对照）：`KissICP.register_frame`（帧到地图 ICP）→ `KissSLAM.process_scan`（累积到局部地图）→ 每走满 `splitting_distance` 触发 `generate_new_node`（切子图 + 回环）→ 最后 `fine_grained_optimization`（密集位姿图优化）。

---

## 1. 前置知识自查表

先诚实地给自己打勾。缺哪块补哪块，不要跳。

| 知识点 | 要求 | 你现在的状态 | 怎么补 |
|---|---|---|---|
| Python 语法/类/模块 | 能写类、能 import | 新手，需补 | 任何入门教程 + 边做边练 |
| numpy 数组 | 熟练：索引/切片/broadcasting/向量化/`np.linalg` | 未接触 | 第 2 节 + 练习 |
| open3d | 会读点云/体素下采样/KD-tree/可视化 | 未接触 | 第 2 节 + 练习 |
| 线性代数 | 矩阵乘/逆/转置/特征值分解 | 十四讲够用 | 巩固向量化思维 |
| 刚体变换 SE(3) | 旋转矩阵、齐次坐标、4×4 变换 | 十四讲第 3 章 | 重点复习指数映射 |
| 李群/李代数 so(3)/se(3) | exp/log、扰动求导 | 十四讲第 4 章 | **必学**，ICP 雅可比要用 |
| 最小二乘 / Gauss-Newton | 构造正规方程、迭代求解 | 十四讲第 6 章 | 第 4 阶段练习 |
| 点云基本概念 | 体素、最近邻、法向量、ICP | 未接触 | 结合 open3d 练习 |

> 关键提醒：李代数（se(3) 的指数映射和左扰动求导）是 point-to-plane ICP 和位姿图优化的数学核心，十四讲第 4 章 + 第 6 章的内容务必吃透。

---

## 2. 需要用到的库 & 学到什么程度

环境里已装好（conda env `pyslam`，Python 3.11）：`numpy 1.26.4`、`open3d 0.19.0`、`scipy 1.17.1`、`g2opy`、`pydantic 2.13.4`、`pyquaternion`。

**不要一次学完整个库**，只学到"够用"：

### numpy（重点，占 80% 代码量）
- 创建数组、`dtype`（float32/float64）、`reshape`
- 索引/切片/布尔掩码、`np.unique`、`np.argsort`
- broadcasting（`pts @ R.T + t` 这种一行变换 N 个点）
- 沿轴操作：`np.linalg.norm(axis=)`、`np.sum(axis=)`、`np.mean(axis=)`
- 线性代数：`np.linalg.inv`、`np.linalg.solve`、`np.linalg.lstsq`、`np.linalg.eigh`（求法向要用）
- 资源：《Python 科学计算》numpy 章节，或 numpy 官方 quickstart（几小时即可入门）

### open3d（重点）
- `o3d.io.read_point_cloud` / `o3d.t.io.read_point_cloud`、写点云
- `PointCloud` + `Vector3dVector`，与 numpy 互转（`np.asarray(pcd.points)`）
- `o3d.geometry.PointCloud.voxel_down_sample`（先看它，再自己写）
- `estimate_normals`（看它怎么算法向）
- `o3d.geometry.KDTreeFlann` 的 `search_knn_vector_3d`（ICP 最近邻核心）
- `o3d.visualization.draw_geometries`（调试必用，可视化结果）
- 资源：open3d 官方文档 + 教程

### 其他（辅助）
- `scipy`：稀疏矩阵、`scipy.spatial.cKDTree`（可选，替代 open3d KD-tree）
- `g2opy`：g2o 的 Python 绑定（位姿图优化用，可先用它对照、之后尝试自己写高斯牛顿）
- `pydantic`：只用于配置文件（参考实现用它）。学习期可先用 `dataclass`，最后再对齐
- `pyquaternion`：四元数工具（可选）

---

## 3. 学习路线总览

| 阶段 | 主题 | 产出物 | 大致耗时 |
|---|---|---|---|
| Phase 0 | 环境 + Python/numpy/open3d + 数学热身 | 几个练习脚本 | 3-5 天 |
| Phase 1 | 读懂参考实现 + 数据准备 + 项目骨架 | 能跑通参考 CLI + 项目目录 | 2-3 天 |
| Phase 2 | 手搓 KISS-ICP 里程计（核心） | 自己的 odometry，轨迹与官方一致 | 1-2 周 |
| Phase 3 | 手搓 KISS-SLAM（子图/位姿图/回环） | 自己的 SLAM，能检测回环 | 1-2 周 |
| Phase 4 | 集成、评测、与官方对比 | 完整可运行的手搓 SLAM | 3-5 天 |

> 每个阶段结束都有一个"完成标准"（能跑通什么、数值差多少），通过才进下一阶段。

---

## 4. 各阶段详细计划

### Phase 0 —— 环境与基础（别跳过）

**目标**：能用 numpy 写向量化代码，能用 open3d 读点云、下采样、找最近邻、可视化。

**具体练习**（按顺序，每个都要自己敲一遍）：
1. 用 numpy 生成 `(100000, 3)` 的随机点，做 `pts @ R.T + t`（R 是旋转矩阵），验证与循环写法结果一致。
2. 体素索引练习：`idx = np.floor(pts / voxel).astype(np.int32)`，用 `np.unique` 找出每个体素内所有点，`np.mean` 求质心。
3. 用 open3d 读一个 `.ply/.pcd`，转成 numpy 数组，打印 shape、xyz 范围。
4. 调用 `voxel_down_sample`，对比下采样前后点数。
5. 用 `estimate_normals` 算法向，`KDTreeFlann.search_knn_vector_3d` 找一个点的 k 近邻。
6. 数学：手推 se(3) 的指数映射（把 6 维 ξ 映射成 4×4 的 T），写代码验证 `exp(log(T)) ≈ T`。

**完成标准**：练习全部跑通；能不看文档写出 `pts @ R.T + t` 和体素质心。

---

### Phase 1 —— 读懂参考实现 + 数据 + 骨架

**目标**：完全理解"每个 Python 文件在干嘛、哪些算法在 `.so` 里"，并准备好测试数据与项目目录。

**步骤**：
1. **通读参考 `.py`**（它们都很短，是胶水层）：
   - `kiss_icp/kiss_icp.py`（里程计主循环，43 行起）
   - `kiss_icp/voxelization.py` / `mapping.py` / `preprocess.py` / `threshold.py` / `registration.py`（每个对应一个 `.so` 内核）
   - `kiss_slam/slam.py`（SLAM 主循环，61 行起）
   - `kiss_slam/voxel_map.py` / `local_map_graph.py` / `loop_closer.py` / `pose_graph_optimizer.py`
   - `map_closures/map_closures.py`
   - 记录：**哪些逻辑在 Python（可照抄理解）、哪些在 `.so`（要手搓）**。对照 `AGENTS.md` 的"Critical architecture fact"清单。
2. **准备数据**：
   - 首选 KITTI odometry（velodyne `.bin` 格式：每点 4 个 float，`x y z intensity`）。可以只取其中一段（比如序列 00 的前 200 帧）。
   - 如果没有数据，用 `generic` dataloader 直接喂一目录 `.bin/.ply/.pcd` 文件即可。
   - 另外**准备一份合成数据**（见下），用于最开始测 ICP。
3. **跑通参考 CLI**（得到"标准答案"）：
   ```bash
   kiss_slam_pipeline <data-dir> --dataloader generic -n 100 --visualize
   ```
   会输出 `slam_output/latest/`，里面有轨迹 `.g2o`、位姿文件、`trajectory.png`、局部地图 ply 等。这些就是你之后要对比的 ground truth。
4. **搭项目骨架**（目录建议见第 6 节）。

**完成标准**：能用自己的话说出每个模块的输入/输出/作用；参考 CLI 跑通并保留输出；骨架目录建立。

---

### Phase 2 —— 手搓 KISS-ICP 里程计（核心，按依赖顺序）

**目标**：从易到难逐个重写 odometry 的 5 个内核，每个都对照 `.so` 验证，最后组装成 `register_frame`。

| # | 模块 | 参考位置 | 算法要点 | 难度 |
|---|---|---|---|---|
| 2.1 | `voxel_down_sample` | `kiss_icp.pybind._voxel_down_sample` | 体素索引 → `np.unique` → 每体素质心 | ★ |
| 2.2 | `VoxelHashMap` | `kiss_icp.pybind._VoxelHashMap` | 哈希表存体素；`add_points` 加已配准点、`update(points, pose)` 变换后插入并剔除远点、`point_cloud` 取所有点 | ★★★ |
| 2.3 | `AdaptiveThreshold` | `kiss_icp.pybind._AdaptiveThreshold` | 用 model_deviation 自适应调 ICP 距离阈值 | ★★ |
| 2.4 | `Preprocessor` | `kiss_icp.pybind._Preprocessor` | 按 max/min range 裁剪；deskew（按时间戳线性插值运动补偿） | ★★★ |
| 2.5 | `Registration` | `kiss_icp.pybind._Registration` | point-to-plane ICP：KD-tree 最近邻 → 构造 6×6 正规方程 → 高斯牛顿迭代 + 鲁棒核 | ★★★★★ |
| 2.6 | 组装 `KissICP` | `kiss_icp/kiss_icp.py` | 把上面串起来（deskew → 两次体素化 → 阈值 → ICP → 更新地图/增量/位姿） | ★ |

**建议顺序与做法**：

1. **2.1 体素下采样**（半天）。完全用 numpy，参考 `voxelization.py` 用法。
2. **2.5 ICP**（先做，因为它是灵魂，且能独立测试）：先用**合成数据**测——生成一个点云，施加已知变换 T，看你的 ICP 能否恢复到 T。数学参考：point-to-plane 残差 `(T·p - q)·n_q`，用 se(3) 左扰动求雅可比 `[n^T, ((T·p)×n)^T]`，6×6 正规方程求解。最近邻用 open3d `KDTreeFlann`。
3. **2.2 / 2.3 / 2.4** 逐个补齐，每个都用 `.so` 对照。
4. **2.6 组装**，用真实 KITTI 数据跑，和官方 `kiss_icp_pipeline` 输出轨迹对比（KITTI 格式位姿文件直接 `np.loadtxt` 对比）。

**完成标准**：`kiss_icp_pipeline` 官方轨迹 vs 你的轨迹，逐帧位姿误差在可接受范围（初期允许 cm 级，逐步逼近）；里程计整体能稳定跑完一段序列不漂飞。

---

### Phase 3 —— 手搓 KISS-SLAM（在里程计之上）

**目标**：在能跑的里程计上，加局部地图管理、位姿图、回环。

| # | 模块 | 参考位置 | 算法要点 | 难度 |
|---|---|---|---|---|
| 3.1 | `VoxelMap` | `kiss_slam_pybind._VoxelMap` | 每个体素存"点 + 法向"（协方差矩阵最小特征值对应法向）；`integrate_frame` 变换后按体素更新；`point_cloud`/`num_voxels`/`open3d_pcd_with_normals` | ★★★ |
| 3.2 | `LocalMapGraph` | `kiss_slam/local_map_graph.py` | 纯 Python 数据结构（子图节点、keypose、local_trajectory）——**直接照抄理解** | ★ |
| 3.3 | `PoseGraphOptimizer` | `kiss_slam_pybind._PoseGraphOptimizer` (g2o) | 位姿图：固定首节点、边为相对位姿、`np.eye(6)` 信息矩阵。**先用 g2opy 复现，之后可手写 se(3) 高斯牛顿** | ★★★★★ |
| 3.4 | `LoopCloser` | `map_closures` + `kiss_slam/loop_closer.py` | 密度图降维 → 描述子(HBST) → 汉明距离匹配 → ICP 精配准 → 重叠率验证(>0.4) | ★★★★★ |
| 3.5 | 组装 `KissSLAM` | `kiss_slam/slam.py` | `process_scan` / `generate_new_node` / `fine_grained_optimization`（胶水，对照理解） | ★★ |
| 3.6 | `OccupancyMapper` | `kiss_slam_pybind._OccupancyMapper` | 占用栅格（选做，最后再做） | ★★★ |

**建议顺序**：
1. 3.2 `LocalMapGraph`（纯 Python，先搞懂子图如何切分、keypose 怎么算）。
2. 3.1 `VoxelMap`（法向用 `np.linalg.eigh` 求协方差最小特征向量）。
3. 3.3 位姿图：先用 g2opy 复现官方行为（对照 `.g2o` 输出），理解"固定首节点 + 相对边 + 单位信息矩阵"后，再尝试自己写一个 se(3) 上的高斯牛顿优化器。
4. 3.4 回环（最独立也最难）：建议先做**简化版**（比如用全局描述子做粗匹配 + ICP 验证），跑通闭环逻辑后再实现完整 density-map/HBST。
5. 3.5 组装，跑通 `kiss_slam_pipeline` 全流程。
6. 3.6 占用栅格选做。

**完成标准**：你的 SLAM 能检测到回环（对比官方 `trajectory.png` 里的红色闭环连线）、闭环后轨迹漂移被拉回；位姿图优化后的轨迹与官方基本一致。

---

### Phase 4 —— 集成、评测与对比

**目标**：把整个系统打磨成"可运行、可评测"的完整项目。

1. 把里程计 + SLAM 串成一条完整 pipeline，接口对齐官方（`process_scan(frame, timestamps)` → `poses`）。
2. **评测**：写脚本计算 ATE / RPE（可用 `kiss_icp/metrics.py` 里的公式参考），与官方实现跑同一段数据对比误差。
3. **可视化**：用 open3d 把轨迹、局部地图、回环画出来（官方 `tools/visualizer.py` 可参考）。
4. 整理成自己的 CLI 或脚本入口，写个简短 README 说明怎么跑。

**完成标准**：一条命令跑完"读数据 → 里程计 → 回环 → 位姿图 → 输出轨迹"，数值与官方在合理范围内，项目结构清晰可读。

---

## 5. "对照验证"方法（贯穿全程的核心技巧）

每个手搓模块，都这样验证：

```python
import numpy as np
from kiss_icp.pybind import kiss_icp_pybind  # 官方 C++ 内核
import my_kiss  # 你自己的实现

pts = np.random.rand(100000, 3).astype(np.float64) * 10.0

# 官方实现（C++）
ref = np.asarray(
    kiss_icp_pybind._voxel_down_sample(
        kiss_icp_pybind._Vector3dVector(pts), 1.0
    )
)
# 你的实现
mine = my_kiss.voxel_down_sample(pts, 1.0)

# 比对
print(ref.shape, mine.shape)
```

**必须注意的坑**：

1. **输出顺序可能不同**：体素下采样/哈希表返回的"点集合"顺序取决于内部哈希遍历顺序，两种实现往往不同。**不能直接 `np.allclose(ref, mine)`**，要先把两者都 `np.sort` 后再比，或按"集合"比对（每个点找最近邻，误差 < 1e-6 才算匹配）。
2. **精度**：官方 `voxel_down_sample` 用 double（`_Vector3dVector`），但 `VoxelMap` 用 float32（`_Vector3fVector`）。对照时保持 dtype 一致，否则会有 `1e-6` 量级差异，属正常。
3. **随机性**：ICP 等算法对输入顺序/初始化敏感，比对轨迹用**整体误差**（逐帧位姿差），不要逐点比。
4. **先测小再测大**：先在合成数据/100 帧上验证，再上完整序列。

---

## 6. 建议项目目录结构

```
KISS_SLAM/
├── AGENTS.md                # 已存在，参考实现位置/架构备忘
├── LEARNING_ROADMAP.md      # 本文件
├── README.md                # 项目说明 + 怎么跑
├── data/                    # 测试数据（KITTI 或合成，gitignore）
├── src/
│   └── kiss_slam_handroll/
│       ├── config.py        # 配置（先用 dataclass）
│       ├── voxelization.py  # Phase 2.1
│       ├── voxel_hash_map.py# Phase 2.2
│       ├── threshold.py     # Phase 2.3
│       ├── preprocess.py    # Phase 2.4
│       ├── icp.py           # Phase 2.5
│       ├── odometry.py      # Phase 2.6
│       ├── voxel_map.py     # Phase 3.1
│       ├── local_map_graph.py # Phase 3.2
│       ├── pose_graph.py    # Phase 3.3
│       ├── loop_closure.py  # Phase 3.4
│       └── slam.py          # Phase 3.5
├── scripts/
│   ├── run_slam.py          # 主入口
│   └── compare_with_ref.py  # 对照验证脚本
└── tests/                   # 每个模块的单元测试
```

---

## 7. 资源汇总

- **数学/理论**：《视觉 SLAM 十四讲》第 3、4、6 章（刚体变换、李群李代数、非线性优化）；KISS-ICP 原论文（"KISS-ICP: In Defense of Point-to-Point ICP"）与 KISS-SLAM 论文（安装在 `kiss_slam-0.0.2.dist-info/METADATA` 里有链接）。
- **numpy**：numpy 官方 `Quickstart tutorial` + 《Python 科学计算》。
- **open3d**：官方 docs 的 `voxel_down_sample` / `KDTreeFlann` / `estimate_normals` / `draw_geometries` 条目。
- **ICP**：搜索 "point-to-plane ICP" 推导（残差 + 雅可比 + 高斯牛顿），对照十四讲第 7 章（ICP）第 6 章（优化）。
- **位姿图/g2o**：十四讲第 10、11 章（后端优化）；g2opy 的 README 示例。
- **回环检测**：KISS-SLAM 论文的 loop-closure 章节；可先了解 Scan Context / 密度图描述子的思路。

---

## 8. 学习节奏与常见坑

**节奏建议**：
- 每天先学一点库用法，再立刻写代码，不要"先看完整个文档"。
- 每个模块遵循"理解算法 → 写实现 → 对照 `.so` 验证 → 记录误差"四步，别急着往后赶。
- ICP 和位姿图是两道坎，卡住时回到十四讲把李代数推导亲手推一遍。

**常见坑（新手高发）**：
1. **变换方向搞反**：本项目约定是行向量 `pcd @ R.T + t`（点右乘矩阵），和"列向量左乘"是反的。所有变换、位姿合成（`keypose @ rel_pose`）都要和官方保持一致，否则轨迹会明显错。
2. **忽略坐标系/单位**：输入点云单位（米）、max_range 单位，体素大小要和官方默认对齐（`mapping.voxel_size = max_range/100`）。
3. **精度不匹配**：float32/float64 混用导致微小的数值差，先统一成官方口径。
4. **一上来就用大数据**：先用合成数据 + 少量帧调通，再上完整序列。
5. **只看不写**：手搓项目核心是"写"，每个模块都必须自己敲代码并对照验证，光读源码是学不会的。

---

## 附：一句话总览每个阶段做什么

- **Phase 0**：练熟 numpy 向量化和 open3d 基础。
- **Phase 1**：读懂参考、备好数据、跑通官方得到标准答案。
- **Phase 2**：手搓 KISS-ICP 的 5 个内核并逐一对拍，得到能跑的里程计。
- **Phase 3**：加上局部地图、位姿图、回环，得到完整 SLAM。
- **Phase 4**：集成、评测、可视化，产出完整项目。
