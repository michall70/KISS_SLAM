import numpy as np
from print_log import print_log

class VoxelHashMap:
    def __init__(self, voxel_size: float, max_distance: float | None = None):
        self.voxel_size = voxel_size
        self.max_distance = max_distance
        self.internal_map = {}

    def add_points(self, points: np.ndarray):
        from functions import voxel_down_sample
        sample, indices = voxel_down_sample(points, self.voxel_size, return_voxel=True)
        N = np.shape(sample)[0]
        for i in range(N):
            self.internal_map[tuple(indices[i])] = tuple(sample[i])

    def update(self, points: np.ndarray, pose: np.ndarray = np.eye(4)):
        from functions import transform_points
        points_new = transform_points(points, pose)
        origin = pose[:3, 3].ravel()
        self.add_points(points_new)
        self.remove_faraway_points(origin)

    def remove_faraway_points(self, origin: np.ndarray = [0, 0, 0]):
        for voxel in list(self.internal_map.keys()):    ## 必须转为 list，否则遍历时修改字典会报错
            t = np.array(self.internal_map[voxel]) - origin
            distance = np.linalg.norm(t)
            if self.max_distance is not None:
                if distance > self.max_distance:
                    del self.internal_map[voxel]

    def to_points(self):
        if not self.internal_map:
            print_log("warn", "internal_map has 0 points")
            return np.empty((0, 3), dtype=np.float64)
        return np.array(tuple(self.internal_map.values()), dtype=np.float64)

    def to_pointcloud(self, color:list | None = None):
        from functions import to_pcd
        return to_pcd(self.to_points(), color)

if __name__ == "__main__":
    from functions import exp_map, read_velodyne_dir, read_kitti_bin
    import open3d as o3d

    map = VoxelHashMap(0.1, 10)

    # flat plane ----------------------------------------
    # x = np.linspace(-10, 10, 200)
    # y = np.linspace(-10, 10, 200)
    # X, Y = np.meshgrid(x, y)
    # Z = np.zeros_like(X)
    # pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis = 1)

    # map.update(pts)
    # pts_get = map.to_pointcloud()
    # o3d.visualization.draw_geometries([pts_get], window_name="Open3D")

    # xi = [5, 0, 0,  0, 0, 0]
    # T = exp_map(xi)
    # map.update(pts, T)
    # pts_get = map.to_pointcloud()
    # o3d.visualization.draw_geometries([pts_get], window_name="Open3D")

    bin_file = "/media/michall/学习资料/Michall/datasets/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data/0000000000.bin"
    pts = read_kitti_bin(bin_file)
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
    # print(np.shape(pts))
    for x in range(0, 50, 5):
        t = [x, 0, 0]
        map.add_points(pts)
        map.remove_faraway_points(t)

        # pose = np.eye(4)
        # pose[0, 3] = x
        # map.update(pts, pose)

        map_pts = map.to_points()
        print(f"x = {x}, pts:{np.shape(map_pts)}")
        pcd = map.to_pointcloud()
        o3d.visualization.draw_geometries([pcd, coordinate_frame], window_name="Open3D")

