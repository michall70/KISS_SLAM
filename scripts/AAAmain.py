"""KISS-SLAM 手搓版 - 里程计运行入口。

先询问配置文件(只加载轻量库), 再加载 open3d 等重模块, 让 prompt 快速出现。
"""
from pathlib import Path

from config_loader import load_config, CONFIG_DIR


def main():
    # ----- 先询问配置文件(此时未加载 open3d, 响应快) -----
    # available = sorted(CONFIG_DIR.glob("*.yaml"))
    # print("可用的配置文件:")
    # for f in available:
    #     print(f"  {f.name}")

    while True:
        user_path = input("请输入配置文件文件名 (直接回车使用默认 config.yaml): ").strip()
        try:
            config = load_config(user_path or None)
            break
        except FileNotFoundError:
            print(f"找不到配置文件: {user_path or 'config.yaml'}, 请从上方列表选择或直接回车")
    data_cfg = config["data"]

    # ----- 之后才加载重模块(open3d import 约 3.7s) -----
    import open3d as o3d
    from print_log import print_log
    from functions import read_point_cloud, create_grid_ground
    from visualize import save_cost_plot, save_open3d_geometries
    from preprocessor import get_timestamps
    from odometry import Odemetry

    # ----- 路径从配置读取 -----
    dir_path = Path(data_cfg["data_dir"])
    ts_start_file = data_cfg["timestamps_start_file"]
    ts_end_file = data_cfg["timestamps_end_file"]

    # 时间戳文件可选: 配置里为 None/不存在 则跳过 deskew
    if ts_start_file and ts_end_file:
        ts_start_path = Path(ts_start_file)
        ts_end_path = Path(ts_end_file)
        ts_available = ts_start_path.exists() and ts_end_path.exists()
    else:
        ts_available = False

    # 检查路径存在性(时间戳文件可选)
    missing = []
    if not dir_path.exists():
        missing.append("bin数据目录")
    if missing:
        print_log("error", f"以下路径不存在: {', '.join(missing)}")
        return

    if not ts_available:
        print_log("warn", "时间戳文件缺失, timestamps 将置为 None (跳过 deskew)")

    # 获取并排序点云文件(.bin 或 .npy)
    point_files = sorted(p for p in dir_path.iterdir() if p.suffix.lower() in (".bin", ".npy"))
    if not point_files:
        print_log("warn", "未找到任何 .bin / .npy 点云文件")
        return

    # 限制文件数量（调试用，配置里 max_files 设为 None 则不限制）
    MAX_FILES = data_cfg["max_files"]
    if MAX_FILES is not None and MAX_FILES > 0:
        if MAX_FILES < len(point_files):
            point_files = point_files[:MAX_FILES]
            print_log("info", f"调试模式：仅处理前 {MAX_FILES} 个文件（共 {len(point_files)} 个）")
        else:
            print_log("info", f"MAX_FILES({MAX_FILES}) 大于实际文件数({len(point_files)})，将处理全部文件")
    else:
        print_log("info", f"未限制数量，将处理全部 {len(point_files)} 个文件")

    # 创建输出目录
    output_root = Path("./output")
    open3d_dir = output_root / "open3d"
    plt_dir = output_root / "plt"
    open3d_dir.mkdir(parents=True, exist_ok=True)
    plt_dir.mkdir(parents=True, exist_ok=True)

    # 初始化里程计对象与公共几何体
    myOdometry = Odemetry(config)
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
    grid = create_grid_ground(size=60.0, step=1.0, center=[0, 0, 0])

    # ----- 逐帧处理 -----
    import time
    frame_count = 0
    for idx, file_path in enumerate(point_files):

        if idx % data_cfg["skip_rate"] != 0:   # 抽帧: 每 skip_rate 帧取 1 帧
            continue

        t0 = time.time()
        points = read_point_cloud(file_path)
        if ts_available:
            timestamps = get_timestamps(ts_start_path, ts_end_path, points, idx)
        else:
            timestamps = None

        total_cost = myOdometry.register_frame(points, timestamps)
        local_pcd = myOdometry.local_map.to_pointcloud()

        # 保存当前帧点云 + 坐标系 + 网格的 Open3D 视图
        save_open3d_geometries(
            geometries=[local_pcd, coordinate_frame, grid],
            save_path=open3d_dir / f"frame_{idx:04d}.png"
        )

        # 保存代价收敛曲线
        save_cost_plot(
            cost_sequence=total_cost,
            save_path=plt_dir / f"cost_{idx:04d}.png",
            title=f'ICP Convergence (frame {idx:04d})'
        )

        frame_count += 1
        print_log("info", f"已处理第 {frame_count} 个文件: {file_path.name}，耗时 {time.time()-t0:.3f}s")

    # ----- 处理完成后，保存全局地图 -----
    global_pcd = myOdometry.global_map.to_pointcloud()
    save_open3d_geometries(
        geometries=[global_pcd, coordinate_frame, grid],
        save_path=open3d_dir / "global_map.png"
    )
    print_log("info", "全部处理完成，图像已保存至 ./output 目录")

    # open3d visualize
    o3d.visualization.draw_geometries([global_pcd, coordinate_frame, grid], window_name="Open3D")


if __name__ == "__main__":
    main()
