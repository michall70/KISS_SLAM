from kiss_icp.mapping import VoxelHashMap
from functions import exp_map, transform_points, create_pcd
import numpy as np
import open3d as o3d

map = VoxelHashMap(voxel_size=1.0, max_distance=10.0, max_points_per_voxel=20)

xi = [0.1, 0.1, 0.1,  0.1, 0.1, 0.1]
T = exp_map(xi)

x = np.linspace(-20, 20, 100)
y = np.linspace(-20, 20, 100)
X, Y = np.meshgrid(x, y)
Z = np.zeros_like(X)
pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis = 1)
pts2 = transform_points(pts, T)

pcd = create_pcd(pts, color=[0, 0.7, 0])    # green
pcd2 = create_pcd(pts2, color=[0, 0.7, 0])

# map.remove_far_away_points(pts)
map.add_points(pts)
map.update(pts, T)

map_pts = map.point_cloud()
map_pcd = create_pcd(map_pts, [0.5, 0, 0])    # red


o3d.visualization.draw_geometries([map_pcd], window_name="Open3D")
