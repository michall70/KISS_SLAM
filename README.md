# 手搓 KISS-SLAM (myKISS_SLAM)

一个从零手搓 KISS-ICP / KISS-SLAM 的学习项目。不调用官方 C++ 内核，用 numpy / scipy / open3d 复现算法，并与参考实现对照验证。

> 学习配套文档：
> - [`LEARNING_ROADMAP.md`](LEARNING_ROADMAP.md) — 从入门到手搓的完整学习路线
> - [`AGENT.md`](AGENT.md) — 项目状态、模块结构、踩坑记录（给 AI 协作看）

## 当前状态

- ✅ **KISS-ICP 里程计已跑通**：预处理 → 体素化 → 自适应阈值 → 向量化 point-to-plane ICP → 局部地图 → `register_frame` 完整里程计。在 KITTI 数据上效果好、速度快。
- ⬜ KISS-SLAM 层（局部地图切分 / 回环检测 / 位姿图优化）尚未开始。

## 环境

- conda 环境：`pyslam`（Python 3.11）
- 依赖：`numpy`、`open3d`、`scipy`、`PyYAML`
- 系统 `python` 没有这些包，务必用 conda 环境

```bash
conda activate pyslam
```

## 运行

```bash
cd scripts
python AAAmain.py
```

运行后询问配置文件文件名，**直接回车用默认 `config.yaml`**（或输入其他配置名）。程序会先快速出 prompt，再加载 open3d（import 约 3.7s，属正常）。

### 配置文件

配置文件按官方 kiss-icp 风格分类：`data`（裁剪/路径/抽帧）、`mapping`（体素/局部地图半径）、`registration`（ICP 迭代数）、`adaptive_threshold`（自适应阈值）。

- `scripts/config.yaml` — **默认配置**（scripts 下保留，直接回车即可使用）
- `config/` — 含真实数据路径的配置（`kitti_config.yaml`、`mydata_config.yaml`），**已 gitignore 保护隐私**，不提交到 git

换数据集时新建一个 yaml、改 `data_dir` 等参数即可，运行时输入配置文件名（或直接回车用默认），不用改代码。

## 数据

数据路径统一写在配置文件里（`config/` 目录，已 gitignore 保护隐私），不在 README 公开。

- **KITTI raw**：`.bin` 格式（每点 4×float32 `x,y,z,intensity`），配套 `timestamps_start.txt` / `timestamps_end.txt`。
- **npy 数据**：`(N,3)` 二维数组。

`read_point_cloud` 会自动按扩展名读 `.bin` / `.npy`。

## 手搓的模块（`scripts/`）

| 文件 | 功能 |
|---|---|
| `AAAmain.py` | 运行入口（先询问配置，再加载 open3d） |
| `odometry.py` | `Odemetry` 类，`register_frame` 组装完整里程计 |
| `functions.py` | 公共函数库：变换、exp_map、读点云、体素下采样等 |
| `icp_synthetic.py` | 向量化 point-to-plane ICP（scipy cKDTree + 鲁棒核） |
| `VoxelHashMap.py` | 局部地图（dict 实现，每体素 1 点） |
| `threshold.py` | 自适应阈值（EMA） |
| `preprocessor.py` | 距离裁剪 + deskew（运动补偿） |
| `config_loader.py` | yaml 配置加载 |
| `visualize.py` / `print_log.py` | 可视化 / 日志工具 |

## 目录

```
KISS_SLAM/
├── scripts/            # 手搓代码
│   ├── AAAmain.py      # 运行入口
│   ├── config.yaml     # 默认配置
│   ├── config_loader.py# 配置加载
│   ├── odometry.py     # 里程计类
│   ├── *.yaml          # 配置文件
│   └── ...             # 各组件模块
├── AGENT.md            # 项目状态与踩坑记录
├── LEARNING_ROADMAP.md # 学习路线
└── README.md
```

## 参考实现

本项目的"判卷老师"（只读，勿改）：`kiss_icp`、`kiss_slam`、`map_closures`，装在 pyslam 环境的 site-packages 里。其核心算法在 C++ pybind11 `.so` 中，手搓时用 numpy/scipy/open3d 复现。
