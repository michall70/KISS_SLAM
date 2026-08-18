import numpy as np
import open3d as o3d

def transform_points(pts, T):
    R = T[0:3, 0:3]
    t_vec = T[0:3, 3]
    return pts @ R.T + t_vec

x = np.linspace(-2, 2, 40)
y = np.linspace(-2, 2, 40)
X, Y = np.meshgrid(x, y)
Z = np.zeros_like(X)

target_pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis = 1)

translation_true = np.array([0.3, 0.2, 0.1])
source_pts = target_pts + translation_true

target_pcd = o3d.geometry.PointCloud()
# source_pcd = o3d.geometry.PointCloud()
