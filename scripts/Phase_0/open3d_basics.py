import numpy as np
import open3d as o3d

def create_grid_ground(size=10.0, step=1.0, center=[0, 0, 0]):
    """
    创建一个网格地面（LineSet）
    :param size:  网格的总边长（米），从 -size/2 到 size/2
    :param step:  网格间距（米）
    :param center: 网格中心位置 [x, y, z]
    :return:       LineSet 对象
    """
    half = size / 2.0
    x0, y0, z0 = center

    # 收集所有线段的端点
    points = []
    lines = []

    # 生成 X 方向的平行线（沿 Y 方向延伸）
    x_vals = np.arange(-half, half + step, step)
    for x in x_vals:
        # 每个 x 处画一条从 (x, -half, 0) 到 (x, half, 0) 的线段
        p1 = [x0 + x, y0 - half, z0]
        p2 = [x0 + x, y0 + half, z0]
        idx1 = len(points)
        points.append(p1)
        idx2 = len(points)
        points.append(p2)
        lines.append([idx1, idx2])

    # 生成 Y 方向的平行线（沿 X 方向延伸）
    y_vals = np.arange(-half, half + step, step)
    for y in y_vals:
        p1 = [x0 - half, y0 + y, z0]
        p2 = [x0 + half, y0 + y, z0]
        idx1 = len(points)
        points.append(p1)
        idx2 = len(points)
        points.append(p2)
        lines.append([idx1, idx2])

    # 创建 LineSet
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(np.array(points))
    line_set.lines = o3d.utility.Vector2iVector(np.array(lines))
    
    # 可选：设置网格线的颜色（灰色）
    line_set.paint_uniform_color([0.7, 0.7, 0.7])  # 浅灰色

    return line_set

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

grid = create_grid_ground(size=10.0, step=1.0, center=[0, 0, 0])
coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
o3d.visualization.draw_geometries([pcd_down, coordinate_frame, grid], window_name="Open3D 点云示例")
print(idx, dist2)