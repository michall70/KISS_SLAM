import numpy as np
import open3d as o3d
import os
import matplotlib.pyplot as plt

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

def read_kitty_bin(file_path):
    raw_data = np.fromfile(file_path, dtype=np.float32, count=-1)
    # num_points = raw_data.size // 4
    # print(f"原始数据元素个数：{raw_data.size}，推测点数：{num_points}")

    points_with_intensity = raw_data.reshape(-1, 4)
    points = points_with_intensity[:, :3]
    return points

def main():
    # # generate target and source ---------------------
    # x = np.linspace(-2, 2, 40)
    # y = np.linspace(-2, 2, 40)
    # X, Y = np.meshgrid(x, y)
    # Z = np.zeros_like(X)

    # target_pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis = 1)

    bin_file = "/media/michall/学习资料/Michall/datasets/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data/0000000000.bin"
    if not os.path.exists(bin_file):
        print("not exist")
        return
    pts = read_kitty_bin(bin_file)
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

    # initialize ------------------------
    target_pcd.estimate_normals(
        search_param = o3d.geometry.KDTreeSearchParamHybrid(radius = 0.2, max_nn = 30)
    )
    target_normals = np.asarray(target_pcd.normals)
    tree = o3d.geometry.KDTreeFlann(target_pcd)
    T = np.eye(4)
    N = np.shape(source_pts)[0]

    # process ----------------------------
    epoch = 5   # iterate number
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