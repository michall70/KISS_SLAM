import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# ===================== 辅助函数（封装保存操作） =====================

def save_open3d_geometries(geometries, save_path, width=640, height=480):
    """
    使用 Open3D 离屏渲染将几何体列表保存为图像文件（默认视角）。
    
    Args:
        geometries (list): o3d.geometry 对象列表（如点云、坐标系、网格等）
        save_path (str or Path): 输出图片的完整路径（支持 png/jpg 等）
        width (int): 渲染窗口宽度（像素）
        height (int): 渲染窗口高度（像素）
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=width, height=height, visible=False)
    for geom in geometries:
        vis.add_geometry(geom)
    # 设置点大小
    opt = vis.get_render_option()
    opt.point_size = 1.0
    # 使用默认视角（不主动调整相机）
    vis.poll_events()
    vis.update_renderer()
    # 捕获屏幕缓冲并转换为图像
    img = vis.capture_screen_float_buffer(do_render=True)
    img_np = (np.asarray(img) * 255).astype(np.uint8)
    o3d.io.write_image(str(save_path), o3d.geometry.Image(img_np))
    vis.destroy_window()


def save_cost_plot(cost_sequence, save_path, title='ICP Convergence', ylabel='Total Cost (sum of r^2)'):
    """
    将代价收敛序列绘制为曲线并保存为图片（不显示窗口）。
    
    Args:
        cost_sequence (list or np.ndarray): 每次迭代的代价数值
        save_path (str or Path): 输出图片路径（推荐 png）
        title (str): 图像标题
        ylabel (str): Y 轴标签
    """
    plt.figure(figsize=(8, 5))
    x = list(range(len(cost_sequence)))
    plt.plot(x, cost_sequence, marker='o')
    for xi, yi in zip(x, cost_sequence):
        plt.annotate(f'{yi:.2f}', (xi, yi), textcoords="offset points", xytext=(0, 5), ha='center', fontsize=7)
    plt.xlabel('Iteration')
    plt.ylabel(ylabel)
    plt.title(title)
    plt.yscale('linear')
    plt.xscale('linear')
    plt.xticks(range(len(cost_sequence)))
    plt.grid(True)
    plt.savefig(save_path, dpi=150)
    plt.close()  # 释放内存