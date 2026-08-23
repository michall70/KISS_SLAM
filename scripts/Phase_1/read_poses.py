import numpy as np

file_path = "/home/michall/AAAProjects/KISS_SLAM/results/2026-08-17_15-19-04/data_poses.npy"
poses = np.load(file_path)

t1 = poses[0, :3, 3]
t2 = poses[1, :3, 3]

delta = np.linalg.norm(t2-t1)
print(delta)