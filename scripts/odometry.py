import numpy as np
from preprocessor import Preprocessor
from VoxelHashMap import VoxelHashMap
from functions import voxel_down_sample
from threshold import AdaptiveThreshold
from icp_synthetic import Kiss_ICP

class Odemetry:
    def __init__(self, config):
        data_cfg = config["data"]
        map_cfg = config["mapping"]
        reg_cfg = config["registration"]
        at_cfg = config["adaptive_threshold"]

        self.voxel_size = map_cfg["voxel_size"]
        self.max_range = data_cfg["max_range"]
        self.min_range = data_cfg["min_range"]
        self.max_distance = map_cfg["max_distance"]
        self.max_epoch = reg_cfg["max_epoch"]
        self.last_delta = np.eye(4)
        self.last_pose = np.eye(4)
        self.model_deviation = np.eye(4)
        self.local_map = VoxelHashMap(self.voxel_size, self.max_distance)
        self.global_map = VoxelHashMap(self.voxel_size, max_distance=None)
        self.get_threshold = AdaptiveThreshold(
            max_range=self.max_range,
            alpha=at_cfg["alpha"],
            min_motion_th=at_cfg["min_motion_th"],
            initial_threshold=at_cfg["initial_threshold"],
        )
        self.preprocessor = Preprocessor(
            max_range=self.max_range,
            min_range=self.min_range,
            deskew=data_cfg["deskew"],
        )

    def register_frame(self, frame: np.ndarray, timestamps: np.ndarray):
        # Preprocess
        frame = self.preprocessor.preprocess(frame, timestamps, self.last_delta)

        # Voxelize
        frame_down = voxel_down_sample(frame, self.voxel_size)

        # first frame
        if not self.local_map.internal_map:
            self.local_map.add_points(frame)
            self.global_map.add_points(frame)
            return [0]  # total cost

        # Get Adaptive Threshold
        sigma = self.get_threshold.compute_threshold(self.model_deviation)

        # initial guess
        initial_guess = self.last_pose @ self.last_delta

        # test initial guess
        print("threshold: ", sigma * 3)
        
        # import open3d as o3d
        # from functions import transform_points, to_pcd
        # local_pcd = self.local_map.to_pointcloud([0, 1, 0])
        # initial_pcd = to_pcd(transform_points(frame_down, initial_guess), [1, 0, 0])
        # coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        # grid = create_grid_ground(size=30.0, step=1.0, center=[0, 0, 0])
        # o3d.visualization.draw_geometries([local_pcd, initial_pcd, coordinate_frame, grid], window_name="Open3D")
        
        # Run ICP
        new_pose, total_cost = Kiss_ICP(
            target_pts = self.local_map.to_points(),
            source_pts = frame_down, 
            initial_guess = initial_guess,
            max_epoch = self.max_epoch, 
            threshold = sigma * 3,
            kernel = sigma,
            return_cost_line=True
            )

        # # test result
        # print("initial guess: \n", initial_guess)
        # print("new pose: \n", new_pose)

        # Update step: threshold, local map, delta, and the last pose
        self.model_deviation = np.linalg.inv(initial_guess) @ new_pose
        self.local_map.update(frame_down, new_pose)
        self.global_map.update(frame_down, new_pose)
        self.last_delta = np.linalg.inv(self.last_pose) @ new_pose
        self.last_pose = new_pose
    
        return total_cost

