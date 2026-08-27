import numpy as np
import open3d as o3d
import os
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from functions import transform_points, exp_map, read_kitti_bin, to_pcd

def Kiss_ICP(target_pts: np.ndarray, source_pts: np.ndarray, initial_guess: np.ndarray = np.eye(4), max_epoch: int = 10, threshold: float = 0.5, kernel: float | None = None, return_cost_line: bool = False):
    '''
    target_pts: 地图点云(世界坐标)
    source_pts: 待配准点云(车体系)
    initial_guess: 初始位姿(恒定速度预测)
    threshold: 对应点距离门槛(3*sigma), 超过的丢弃
    kernel: 鲁棒核尺度(sigma), None 表示不加核
    返回: 配准后的位姿 T (以及可选 cost 曲线)
    '''
    target_pcd = to_pcd(target_pts)
    target_pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30)
    )
    target_normals = np.asarray(target_pcd.normals)

    # scipy cKDTree: 批量查询, 比逐点调用 open3d KDTree 快 1~2 个数量级
    tree = cKDTree(target_pts)

    T = initial_guess
    best_T = T
    best_cost = -1
    if return_cost_line:
        total_cost = [0] * max_epoch

    for round in range(max_epoch):
        p_prime_pts = transform_points(source_pts, T)

        # 批量找最近邻: dist 是每个源点到最近邻的距离, idx 是索引
        dist, idx = tree.query(p_prime_pts, k=1)

        q = target_pts[idx]
        n = target_normals[idx]

        # 对应点距离门槛: 超过 threshold 的点对丢弃
        valid = dist < threshold
        if valid.sum() < 10:
            break

        # 残差 (向量化): r = (p' - q)·n
        r = np.sum((p_prime_pts - q) * n, axis=1)

        # 雅可比 (向量化): J = [n, p' x n], (N,6)
        cross = np.cross(p_prime_pts, n)
        J = np.concatenate([n, cross], axis=1)

        # 鲁棒核权重 (保留原逻辑: 对应点距离 > kernel 用 Geman-McClure, 否则 w=2)
        if kernel is not None and kernel > 0:
            sigma2 = kernel * kernel
            kernel_mask = valid & (dist > kernel)
            w = np.where(kernel_mask, 2.0 * (sigma2 / (sigma2 + r * r)) ** 2, 2.0)
        else:
            w = np.full_like(r, 2.0)

        # 应用门槛: 无效点对权重置 0
        w = w * valid

        # 正规方程 (向量化): H = Σ w_i J_i J_i^T, g = Σ w_i J_i r_i
        H = (w[:, None] * J).T @ J   # (6,6)
        g = (w * r) @ J               # (6,)

        H_reg = H + 1e-6 * np.eye(6)
        delta_xi = np.linalg.solve(H_reg, -g).flatten()
        T = exp_map(delta_xi) @ T

        # cost
        cost = 0.5 * float(np.sum(w * (r ** 2)))
        if best_cost == -1:
            best_cost = cost
            best_T = T
        elif cost < best_cost:
            best_cost = cost
            best_T = T

        if return_cost_line:
            total_cost[round] = cost

    if return_cost_line:
        print("best_cost = ", best_cost)
        return best_T, total_cost
    else:
        return best_T

def main():
    # # generate target and source ---------------------
    # x = np.linspace(-2, 2, 40)
    # y = np.linspace(-2, 2, 40)
    # X, Y = np.meshgrid(x, y)
    # Z = np.zeros_like(X)

    # target_pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis = 1)

    bin_file = "/media/michall/学习资料/Michall/datasets/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data/0000000000.bin"
    if not os.path.exists(bin_file):
        print("bin file is not exist")
        return
    pts = read_kitti_bin(bin_file)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    voxel_size = 0.1
    pcd_down = pcd.voxel_down_sample(voxel_size)
    target_pts = np.asarray(pcd_down.points)

    xi = [0.1, 0.1, 0.1,  0.1, 0.1, 0.1]
    T_true = exp_map(xi)
    source_pts = transform_points(target_pts, T_true)

    target_pcd = o3d.geometry.PointCloud()
    target_pcd.points = o3d.utility.Vector3dVector(target_pts)

    # # visualize initial source -----------------------
    # source_pcd = o3d.geometry.PointCloud()
    # source_pcd.points = o3d.utility.Vector3dVector(source_pts)
    # target_pcd.paint_uniform_color([0.5, 0, 0])
    # source_pcd.paint_uniform_color([0, 0.7, 0])
    # o3d.visualization.draw_geometries([target_pcd, source_pcd], window_name="Open3D")
    # return

    epoch = 20
    T, total_cost = Kiss_ICP(target_pts, source_pts, max_epoch=epoch, threshold=1.5, return_cost_line=True)

    # output ------------------------
    print(np.array2string(T, precision=3, suppress_small=True))

    # 代价收敛曲线
    plt.figure(figsize=(8, 5))
    plt.plot(total_cost, marker='o')
    plt.xlabel('Iteration')
    plt.ylabel('Total Cost (sum of r^2)')
    plt.title('ICP Convergence')
    plt.yscale('log')
    plt.grid(True)
    print(f"Final cost: {total_cost[epoch - 1]}")
    plt.show()

    res_pts = transform_points(source_pts, T)
    res_pcd = o3d.geometry.PointCloud()
    res_pcd.points = o3d.utility.Vector3dVector(res_pts)
    target_pcd.paint_uniform_color([0.5, 0, 0])
    res_pcd.paint_uniform_color([0, 0.7, 0])
    o3d.visualization.draw_geometries([target_pcd, res_pcd], window_name="Open3D")

if __name__ == "__main__":
    main()