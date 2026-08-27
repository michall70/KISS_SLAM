
import numpy as np
import open3d as o3d

from functions import exp_map, to_pcd, read_kitti_bin, rotation_matrix_to_axis_angle


def parse_timestamp(s):
    """解析 '2011-09-26 13:04:32.335337762' -> 秒（浮点）。

    KITTI 时间戳是纳秒精度(9位小数), 而 strptime 的 %f 只支持 6 位微秒,
    所以手动拆开"日期时间主体 + 纳秒小数"分别处理。
    """
    import datetime

    s = s.strip()
    base = datetime.datetime.strptime(s[:19], "%Y-%m-%d %H:%M:%S")
    frac = s[20:]  # 纳秒小数部分(9位), 如 "335337762"
    return base.timestamp() + float("0." + frac)


def get_timestamps(ts_start_file, ts_end_file, points, line_index):
    """
    根据行索引 line_index 读取两个时间戳文件的对应行，
    并返回该帧内每个点的时间戳（线性插值）。
    """
    # 一次性读取两个文件的所有行（时间戳文件通常不大，内存安全）
    with open(ts_start_file) as f:
        start_lines = f.readlines()
    with open(ts_end_file) as f:
        end_lines = f.readlines()

    # 检查行索引是否越界
    if line_index >= len(start_lines) or line_index >= len(end_lines):
        raise IndexError(f"行索引 {line_index} 超出时间戳文件行数")

    # 解析对应行
    ts_start = parse_timestamp(start_lines[line_index])
    ts_end = parse_timestamp(end_lines[line_index])

    # 构造每点时间戳（帧内匀速）
    timestamps = np.linspace(ts_start, ts_end, len(points))
    return timestamps


class Preprocessor:
    def __init__(self, max_range=100.0, min_range=0.0, deskew=True):
        self.max_range = max_range
        self.min_range = min_range
        self.deskew = deskew

    def preprocess(self, frame, timestamps, relative_motion):
        """距离裁剪 + deskew（运动补偿）。

        返回裁剪、去畸变后的点云。
        timestamps 为空或只有一个点时跳过 deskew（比如 KITTI 原始 .bin）。
        """
        points = frame

        # 1. 距离裁剪: 保留范数在 [min_range, max_range] 内的点
        dist = np.linalg.norm(points, axis=1)
        mask = (dist >= self.min_range) & (dist <= self.max_range)
        points = points[mask]

        # 2. deskew: 把点对齐到扫描结束时刻(s=1)
        if self.deskew and timestamps is not None and timestamps.size > 1:
            points = self._deskew(points, timestamps[mask], relative_motion)
        return points

    def _deskew(self, points, timestamps, relative_motion):
        """运动补偿。

        语义: 把每个点从"采集时刻(s_i)的车体系"搬到"扫描结束时刻(s=1)的车体系"。
        - s=0 的点: 应用 inv(relative_motion) 的完全变换(补偿最多)
        - s=1 的点: 不补偿
        - 中间点:   应用 inv(relative_motion) 的 (1-s) 比例(旋转角、平移都线性插值)
        """
        # 时间比例 s ∈ [0,1]
        t0, t1 = timestamps.min(), timestamps.max()
        s = (timestamps - t0) / (t1 - t0)

        # 逆变换: inv(relative_motion), 提取旋转向量和平移
        inv_delta = np.linalg.inv(relative_motion)
        phi_inv = rotation_matrix_to_axis_angle(inv_delta[:3, :3])
        t_inv = inv_delta[:3, 3]

        new_points = np.empty_like(points)
        for i in range(len(points)):
            # 剩余运动 = inv(relative_motion) 的 (1-s) 比例
            rem_pose = exp_map(np.concatenate([(1 - s[i]) * t_inv, (1 - s[i]) * phi_inv]))
            p_h = np.concatenate([points[i], [1.0]])
            new_points[i] = (rem_pose @ p_h)[:3]
        return new_points


if __name__ == "__main__":
    bin_file = "/media/michall/学习资料/Michall/datasets/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data/0000000000.bin"
    ts_start_file = "/media/michall/学习资料/Michall/datasets/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/timestamps_start.txt"
    ts_end_file = "/media/michall/学习资料/Michall/datasets/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/timestamps_end.txt"

    pts = read_kitti_bin(bin_file)

    # 构造每点时间戳
    timestamps = get_timestamps(ts_start_file, ts_end_file, pts, line_index=0)

    # 演示用相对运动(第一帧没有真实 last_delta): 车前进 0.5m + 绕 z 微转 0.02 弧度
    relative_motion = exp_map([0.5, 0, 0, 0, 0, 0.02])

    pre = Preprocessor(max_range=100.0, min_range=0.0, deskew=True)
    deskewed = pre.preprocess(pts, timestamps, relative_motion)

    print(f"原始点数: {len(pts)}, deskew 后点数: {len(deskewed)}")

    # 可视化: 原始(红) vs deskew(绿)
    raw_pcd = to_pcd(pts, [0.8, 0, 0])
    desk_pcd = to_pcd(deskewed, [0, 0.8, 0])
    o3d.visualization.draw_geometries([raw_pcd, desk_pcd], window_name="deskew 前后对比")
