import numpy as np
import open3d as o3d
import os
import matplotlib.pyplot as plt
from functions import transform_points, exp_map, read_kitti_bin, to_pcd

def ICP(target_pts: np.ndarray, source_pts: np.ndarray, epoch: int, return_cost_line: bool = False):
    '''
    epoch: iterate number
    '''
    target_pcd = to_pcd(target_pts)
    # initialize ------------------------
    target_pcd.estimate_normals(
        search_param = o3d.geometry.KDTreeSearchParamHybrid(radius = 0.2, max_nn = 30)
    )
    target_normals = np.asarray(target_pcd.normals)
    tree = o3d.geometry.KDTreeFlann(target_pcd)
    T = np.eye(4)
    N = np.shape(source_pts)[0]

    # process ----------------------------
    if return_cost_line:
        total_cost = [0] * epoch

    for round in range(epoch):
        p_prime_pts = transform_points(source_pts, T)
        H = np.zeros((6, 6))
        g = np.zeros((6, 1))

        for i in range(N):
            p_prime = p_prime_pts[i]
            [_, idx, _] = tree.search_knn_vector_3d(p_prime, 1)
            q = target_pts[idx[0]]
            n = target_normals[idx[0]]
            r = np.dot(p_prime - q, n)
            if return_cost_line:
                total_cost[round] += np.square(r)
            cross = np.cross(p_prime, n)
            J = np.concatenate([n, cross])

            H += np.outer(J, J)
            g += J.reshape(6, 1) * r

        # 给 H 加小量正则化，防止奇异
        H_reg = H + 1e-6 * np.eye(6)
        delta_xi = np.linalg.solve(H_reg, -g).flatten()
        T = exp_map(delta_xi) @ T

    # output ------------------------
    if return_cost_line:
        return T, total_cost
    else:
        return T

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

    epoch = 10
    T, total_cost = ICP(target_pts, source_pts, epoch, return_cost_line=True)

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