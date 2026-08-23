import numpy as np
import open3d as o3d

n_points = 5000
theta = np.random.uniform(0, np.pi, n_points)
phi = np.random.uniform(0, 2 * np.pi, n_points)
x = np.sin(theta) * np.cos(phi)
y = np.sin(theta) * np.sin(phi)
z = np.cos(theta)

pts = np.stack([x, y, z], axis = 1)
pts += 0.02 * np.random.randn(n_points, 3)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pts)

# pts_back = np.asarray(pcd.points)
# print(pts_back.shape)

voxel_size = 0.1
pcd_down = pcd.voxel_down_sample(voxel_size)

print(len(pcd.points))
print(len(pcd_down.points))
print(np.asarray(pcd_down.points))

pcd_down.estimate_normals(
    search_param = o3d.geometry.KDTreeSearchParamHybrid(radius = 0.2, max_nn = 30)
)

normals = np.asarray(pcd_down.normals)
print(normals.shape)

tree = o3d.geometry.KDTreeFlann(pcd_down)
query_point = np.asarray(pcd_down.points)[0]  # 把第一个点拎出来当“队长”
[k, idx, dist2] = tree.search_knn_vector_3d(query_point, 10)
coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
o3d.visualization.draw_geometries([pcd_down, coordinate_frame], window_name="Open3D 点云示例")
print(idx, dist2)