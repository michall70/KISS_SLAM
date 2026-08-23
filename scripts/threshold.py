import numpy as np

class Estimator:
    def __init__(self, max_range=100.0, alpha=0.1, initial_threshold=0.5):
        """
        :param max_range: 雷达最大测距（米），用于将旋转误差换算成米
        :param alpha: EMA 平滑系数 (0 < alpha <= 1)
        :param initial_threshold: 初始阈值（米），作为平滑的初始值
        """
        self.max_range = max_range
        self.alpha = alpha
        self.threshold = initial_threshold  # 当前平滑后的阈值

    def _rotation_matrix_to_angle(self, R: np.ndarray) -> float:
        """
        从 3x3 旋转矩阵计算旋转角度（弧度），基于罗德里格斯公式。
        角度 = arccos((trace(R) - 1) / 2)
        """
        trace = np.trace(R)
        # 数值裁剪，防止因浮点误差导致 trace 超出 [-1, 3] 范围
        cos_theta = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
        angle = np.arccos(cos_theta)
        return angle

    def compute_threshold(self, model_deviation: np.ndarray) -> float:
        """
        输入预测偏差矩阵（4x4），计算并返回平滑后的阈值。
        :param model_deviation: 4x4 齐次变换矩阵，表示 (prediction)^(-1) @ actual
        :return: 平滑后的阈值（米）
        """
        # 1. 提取旋转矩阵和平移向量
        R = model_deviation[:3, :3]   # 3x3
        t = model_deviation[:3, 3]    # 3x1

        # 2. 计算旋转角度（弧度）
        rot_angle = self._rotation_matrix_to_angle(R)

        # 3. 计算平移模长（米）
        trans_norm = np.linalg.norm(t)

        # 4. 计算原始偏差（米）
        #    旋转偏差 = 角度 * max_range (弧长公式)
        rot_deviation = rot_angle * self.max_range
        deviation = max(rot_deviation, trans_norm)

        # 5. EMA 平滑更新阈值
        self.threshold = self.alpha * deviation + (1.0 - self.alpha) * self.threshold

        return self.threshold