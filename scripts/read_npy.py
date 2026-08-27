import numpy as np
import open3d as o3d
from functions import create_grid_ground

file_path = "/media/michall/学习资料/Michall/datasets/2D_to_depth/seq0/pointcloud/000161.npy"
pts = np.load(file_path)
print(np.shape(pts))

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pts)

voxel_size = 0.05
pcd_down = pcd.voxel_down_sample(voxel_size)

grid = create_grid_ground(size=10.0, step=1.0, center=[0, 0, 0])
coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
o3d.visualization.draw_geometries([pcd_down, coordinate_frame, grid], window_name="Open3D")
