import numpy as np
import open3d as o3d
import os

def read_kitty_bin(file_path):
    raw_data = np.fromfile(file_path, dtype=np.float32, count=-1)
    num_points = raw_data.size // 4
    print(f"原始数据元素个数：{raw_data.size}，推测点数：{num_points}")

    points_with_intensity = raw_data.reshape(-1, 4)
    points = points_with_intensity[:, :3]
    return points

def main():
    bin_file = "/media/michall/学习资料/Michall/datasets/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data/0000000000.bin"
    if not os.path.exists(bin_file):
        print("not exist")
        return

    pts = read_kitty_bin(bin_file)
    print("\n=== 坐标范围 ===")
    x_min, x_max = pts[:, 0].min(), pts[:, 0].max()
    y_min, y_max = pts[:, 1].min(), pts[:, 1].max()
    z_min, z_max = pts[:, 2].min(), pts[:, 2].max()
    print(f"X: {x_min:.3f} ~ {x_max:.3f}")
    print(f"Y: {y_min:.3f} ~ {y_max:.3f}")
    print(f"Z: {z_min:.3f} ~ {z_max:.3f}")

    print(pts[:5])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    voxel_size = 0.1
    pcd_down = pcd.voxel_down_sample(voxel_size)
    print(f"pcd: {len(pcd.points)}, pcd_down: {len(pcd_down.points)}")
    o3d.visualization.draw_geometries([pcd_down], window_name="Open3D 点云示例")
    o3d.visualization.draw_geometries([pcd], window_name="Open3D 点云示例")

if __name__ == "__main__":
    main()