import numpy as np

class AdaptiveThreshold:
    def __init__(self, 
                 max_range: float = 100.0, 
                 alpha: float = 0.4, 
                 min_motion_th: float = 0.05, 
                 initial_threshold: float = 0.8):
        """
        KISS-ICP 风格的自适应阈值计算器

        :param max_range:       雷达最大测距（米），用于将旋转弧度换算成米
        :param alpha:           EMA 平滑系数（0.4 实现快跟随，对齐 KISS 默认）
        :param min_motion_th:   最小阈值下限（米），防止静止时阈值归零导致匹配失败
        :param initial_threshold: 初始阈值（米），用于 EMA 的初始状态
        """
        self.max_range = max_range
        self.alpha = alpha
        self.min_motion_th = min_motion_th
        self.threshold = initial_threshold  # 当前存储的平滑阈值
        self.is_first_frame = True

    def _rotation_matrix_to_angle(self, R: np.ndarray) -> float:
        """从 3x3 旋转矩阵提取旋转角度（弧度），范围 [0, pi]"""
        trace = np.trace(R)
        # 数值裁剪，防止浮点误差导致 arccos 输入越界
        cos_theta = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
        return np.arccos(cos_theta)

    def compute_threshold(self, model_deviation: np.ndarray) -> float:
        """
        输入预测偏差矩阵（4x4），返回经过 EMA 平滑并裁剪下限后的阈值

        :param model_deviation: 4x4 齐次矩阵，表示 inv(prediction) @ actual_pose
        :return: 当前帧的自适应阈值（米），保证 >= min_motion_th
        """
        if self.is_first_frame: # don't update when it comes to first frame
            self.is_first_frame = False
            return self.threshold
        
        # 1. 提取旋转和平移分量
        R = model_deviation[:3, :3]
        t = model_deviation[:3, 3]

        # 2. 计算旋转角度（弧度）
        rot_angle = self._rotation_matrix_to_angle(R)

        # 3. 计算平移模长（米）
        trans_norm = np.linalg.norm(t)

        # 4. 计算原始偏差（将旋转换算成米，取两者最大值）
        rot_deviation = rot_angle * self.max_range
        deviation = max(rot_deviation, trans_norm)

        # 5. EMA 平滑（alpha=0.4 实现快速反应）
        self.threshold = self.alpha * deviation + (1.0 - self.alpha) * self.threshold

        # 6. ★ 关键一步：施加下限保护（KISS-ICP 核心机制）
        if self.threshold < self.min_motion_th:
            self.threshold = self.min_motion_th

        return self.threshold

    def reset(self, initial_threshold: float = 0.5):
        """重置内部状态（用于新的 map 或重新初始化）"""
        self.threshold = initial_threshold