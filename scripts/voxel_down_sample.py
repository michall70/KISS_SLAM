import numpy as np

def voxel_down_sample(pts, voxel_size):
    '''
    points: (N, 3)
    return: 每个被占据体素的第一个点
    '''

    voxel_indices = np.floor(pts / voxel_size).astype(np.int32)
    unique_voxels, first_indices = np.unique(voxel_indices, axis = 0, return_index=True)
    sample = pts[first_indices]
    return sample

def main():
    import open3d as o3d
    from kiss_icp.pybind import kiss_icp_pybind

    voxel_size = 0.23

    x = np.linspace(-2, 2, 100)
    y = np.linspace(-2, 2, 100)
    X, Y = np.meshgrid(x, y)
    Z = np.zeros_like(X)
    pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis = 1)

    # reference
    ref_pcd = kiss_icp_pybind._Vector3dVector(pts)
    ref_result = kiss_icp_pybind._voxel_down_sample(ref_pcd, voxel_size)
    ref_centroids = np.asarray(ref_result)  # (M_ref, 3)
    ref = o3d.geometry.PointCloud()
    ref.points = o3d.utility.Vector3dVector(ref_centroids)
    ref.paint_uniform_color([0.5, 0, 0])

    pts_down = voxel_down_sample(pts, voxel_size)
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(pts)
    # pcd.paint_uniform_color([0.5, 0, 0])
    pcd_down = o3d.geometry.PointCloud()
    pcd_down.points = o3d.utility.Vector3dVector(pts_down)
    pcd_down.paint_uniform_color([0, 0.7, 0])
    o3d.visualization.draw_geometries([ref, pcd_down], window_name="Open3D")

if __name__ == "__main__":
    main()