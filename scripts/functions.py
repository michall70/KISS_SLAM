# 存放常用function

# transform_points()
# exp_map()
# rotation_matrix_to_axis_angle()

# read_kitti_bin()
# read_velodyne_dir()

# to_pcd()
# voxel_down_sample()
# create_grid_ground()

import numpy as np
import open3d as o3d
from print_log import print_log

def transform_points(pts, T):
    R = T[0:3, 0:3]
    t_vec = T[0:3, 3]
    return pts @ R.T + t_vec

def exp_map(xi):
    '''
    xi must be a row vector with 6 elements
    xi = [rho, phi]
    '''

    rho = xi[0:3]
    phi = xi[3:6]
    theta = np.linalg.norm(phi)
    t = rho     # approximately
    mat_phi = np.array(
        [[0, -phi[2], phi[1]],
         [phi[2], 0, -phi[0]],
         [-phi[1], phi[0], 0]]
    )

    if theta <= 1e-6:
        R = np.eye(3) + mat_phi
    else:
        mat_phi_sq = mat_phi @ mat_phi
        R = np.eye(3) + (np.sin(theta) / theta) * mat_phi + ((1 - np.cos(theta)) / np.square(theta)) * mat_phi_sq

    # print(R @ R.transpose())
    T = np.eye(4)
    T[:3, :3] = R       # 左上 3×3 放旋转
    T[:3, 3]  = t       # 右上放平移
    return T

def rotation_matrix_to_axis_angle(R):
    """3x3 旋转矩阵 -> 旋转向量（轴角）。

    公式: theta = arccos((trace(R) - 1) / 2), 轴 = (R - R^T) 的反对称部分 / (2 sin(theta))
    """
    theta = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    if theta < 1e-8:
        return np.zeros(3)
    v = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ])
    axis = v / (2.0 * np.sin(theta))
    return axis * theta

def read_kitti_bin(file_path):
    import os
    if not os.path.exists(file_path):
        print_log('error', "bin file is not exist")
        return
    raw_data = np.fromfile(file_path, dtype=np.float32, count=-1)
    # num_points = raw_data.size // 4
    # print(f"原始数据元素个数：{raw_data.size}，推测点数：{num_points}")

    points_with_intensity = raw_data.reshape(-1, 4)
    points = points_with_intensity[:, :3]
    return points

def read_velodyne_dir(dir_path):
    from pathlib import Path
    bin_names = []
    # 只匹配 *.bin
    for file_path in dir_path.glob("*.bin"):
        bin_names.append(file_path.name)
    return bin_names

def to_pcd(points: np.ndarray, color:list | None = None):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    if color is not None:
        point_cloud.paint_uniform_color(color)
    return point_cloud

def voxel_down_sample(pts: np.ndarray, voxel_size: float, return_voxel: bool = False):
    '''
    points: (N, 3)
    return: 
        sample: 最终保留的点
        unique_voxel: 体素索引，是整数，经过放缩，非实际坐标。
    '''

    voxel_indices = np.floor(pts / voxel_size).astype(np.int32)
    unique_voxel, first_indices = np.unique(voxel_indices, axis = 0, return_index=True)
    sample = pts[first_indices]
    if not return_voxel:
        return sample
    return sample, unique_voxel

def create_grid_ground(size=10.0, step=1.0, center=[0, 0, 0]):
    """
    创建一个网格地面（LineSet）
    :param size:  网格的总边长（米），从 -size/2 到 size/2
    :param step:  网格间距（米）
    :param center: 网格中心位置 [x, y, z]
    :return:       LineSet 对象
    """
    half = size / 2.0
    x0, y0, z0 = center

    # 收集所有线段的端点
    points = []
    lines = []

    # 生成 X 方向的平行线（沿 Y 方向延伸）
    x_vals = np.arange(-half, half + step, step)
    for x in x_vals:
        # 每个 x 处画一条从 (x, -half, 0) 到 (x, half, 0) 的线段
        p1 = [x0 + x, y0 - half, z0]
        p2 = [x0 + x, y0 + half, z0]
        idx1 = len(points)
        points.append(p1)
        idx2 = len(points)
        points.append(p2)
        lines.append([idx1, idx2])

    # 生成 Y 方向的平行线（沿 X 方向延伸）
    y_vals = np.arange(-half, half + step, step)
    for y in y_vals:
        p1 = [x0 - half, y0 + y, z0]
        p2 = [x0 + half, y0 + y, z0]
        idx1 = len(points)
        points.append(p1)
        idx2 = len(points)
        points.append(p2)
        lines.append([idx1, idx2])

    # 创建 LineSet
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(np.array(points))
    line_set.lines = o3d.utility.Vector2iVector(np.array(lines))
    
    # 可选：设置网格线的颜色（灰色）
    line_set.paint_uniform_color([0.7, 0.7, 0.7])  # 浅灰色

    return line_set

