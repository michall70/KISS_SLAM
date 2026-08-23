import numpy as np
from preprocessor import Preprocessor, get_timestamps
from VoxelHashMap import VoxelHashMap
from functions import voxel_down_sample
from threshold import AdaptiveThreshold
from icp_synthetic import Kiss_ICP

class Odemetry:
    def __init__(self):
        self.voxel_size = 0.1
        self.max_range = 100    # for preprocess
        self.min_range = 0.0
        self.max_distance = 30  # for localmap
        self.last_delta = np.eye(4)
        self.last_pose = np.eye(4)
        self.model_deviation = np.eye(4)
        self.local_map = VoxelHashMap(self.voxel_size, self.max_distance)
        self.global_map = VoxelHashMap(self.voxel_size, max_distance=None)

    def register_frame(self, frame: np.ndarray, timestamps: np.ndarray):
        # Preprocess
        preprocessor = Preprocessor(self.max_range, self.min_range)
        preprocessor.preprocess(frame, timestamps, self.last_delta)

        # Voxelize
        frame_down = voxel_down_sample(frame, self.voxel_size)

        # Get Adaptive Threshold
        get_threshold = AdaptiveThreshold()
        sigma = get_threshold.compute_threshold(self.model_deviation)

        # initial guess
        initial_guess = self.last_pose @ self.last_delta

        # first frame
        if not self.local_map.internal_map:
            self.local_map.add_points(frame)
            self.global_map.add_points(frame)
            return

        # test initial guess
        import open3d as o3d
        from functions import transform_points, to_pcd
        local_pcd = self.local_map.to_pointcloud([0, 1, 0])
        initial_pcd = to_pcd(transform_points(frame_down, initial_guess), [1, 0, 0])
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        o3d.visualization.draw_geometries([local_pcd, initial_pcd, coordinate_frame], window_name="Open3D")
        
        # Run ICP
        new_pose = Kiss_ICP(
            target_pts = self.local_map.to_points(),
            source_pts = frame_down, 
            max_epoch = 20, 
            threshold = sigma * 3,
            # kernel = sigma,
            return_cost_line=True
            )

        # Update step: threshold, local map, delta, and the last pose
        self.model_deviation = np.linalg.inv(initial_guess) @ new_pose
        self.local_map.update(frame_down, new_pose)
        self.global_map.update(frame_down, new_pose)
        self.last_delta = np.linalg.inv(self.last_pose) @ new_pose
        self.last_pose = new_pose
    
        return


def main():
    from functions import read_kitti_bin
    from pathlib import Path
    from print_log import print_log
    import open3d as o3d

    dir_path = "/media/michall/学习资料/Michall/datasets/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data/"
    ts_start_file = "/media/michall/学习资料/Michall/datasets/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/timestamps_start.txt"
    ts_end_file = "/media/michall/学习资料/Michall/datasets/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/timestamps_end.txt"

    dir_path = Path(dir_path)
    ts_start_path = Path(ts_start_file)
    ts_end_path = Path(ts_end_file)

    # ----- 1. 同时检测三个路径是否存在 -----
    missing = []
    if not dir_path.exists():
        missing.append("数据目录")
    if not ts_start_path.exists():
        missing.append("timestamps_start.txt")
    if not ts_end_path.exists():
        missing.append("timestamps_end.txt")

    if missing:
        print_log("error", f"以下路径不存在: {', '.join(missing)}")
        return

    # ----- 2. 按文件名排序获取所有 .bin 文件（保证顺序与时间戳行对应）-----
    # 注意：使用 glob("*.bin") 只取当前目录下的 .bin，不递归（若需递归请改为 rglob）
    bin_files = sorted(dir_path.glob("*.bin"))
    if not bin_files:
        print_log("warning", "未找到任何 .bin 文件")
        return
    
    # 应用限制
    MAX_FILES = 3

    if MAX_FILES is not None and MAX_FILES > 0:
        # 如果 MAX_FILES 小于实际文件数，只取前 N 个
        if MAX_FILES < len(bin_files):
            bin_files = bin_files[:MAX_FILES]
            print_log("info", f"调试模式：仅处理前 {MAX_FILES} 个文件（共 {len(bin_files)} 个）")
        else:
            print_log("info", f"MAX_FILES({MAX_FILES}) 大于实际文件数({len(bin_files)})，将处理全部文件")
    else:
        print_log("info", f"未限制数量，将处理全部 {len(bin_files)} 个文件")

    myOdometry = Odemetry()
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])


    for idx, file_path in enumerate(bin_files):  # idx 从 0 开始
        points = read_kitti_bin(file_path) 
        timestamps = get_timestamps(ts_start_path, ts_end_path, points, idx)

        myOdometry.register_frame(points, timestamps)
        print_log("info", f"已处理第 {idx+1} 个文件: {file_path.name}")
        local_pcd = myOdometry.local_map.to_pointcloud()
        o3d.visualization.draw_geometries([local_pcd, coordinate_frame], window_name="Open3D")

    global_pcd = myOdometry.global_map.to_pointcloud()
    o3d.visualization.draw_geometries([global_pcd, coordinate_frame], window_name="Open3D")

if __name__ == "__main__":
    main()