import numpy as np
import time

N = 10000
t = [1, 2, 3]
pts = np.random.rand(N, 3) * 10

pts_1 = pts.copy()  # !!! numpy默认是引用，不是copy !!!
start = time.perf_counter()
for i in range(N):
    for j in range(3):
        pts_1[i][j] += t[j]
end = time.perf_counter()
elapse_1 = end - start

start = time.perf_counter()
pts_2 = pts.copy() + t
end = time.perf_counter()
elapse_2 = end - start

print("elapse_1 : ", elapse_1)
print("elapse_2 : ", elapse_2)

# -----------------------------------

Rz = np.array([
    [0, 1, 0],
    [-1, 0, 0],
    [0, 0, 1]
])
print("Rz =\n", Rz)
print("\n验证正交性: Rz @ Rz.T =\n", Rz @ Rz.T)

new_pts = pts @ Rz.T
print("length_1 = ", np.linalg.norm(pts[1]))
print("length_2 = ", np.linalg.norm(new_pts[1]))

# -----------------------------------

def transform_points(pts, T):
    R = T[0:3, 0:3]
    t_vec = T[0:3, 3]
    return pts @ R.T + t_vec

T = np.eye(4)
T[0:3, 0:3] = Rz
T[0:3, 3] = t

pts_transformed = transform_points(pts, T)
print(f"变换前点云形状: {pts.shape}")
print(f"变换后点云形状: {pts_transformed.shape}")

# 顺便验证一下前几个点是否与旋转 + 平移一致
print("\n前 3 个点变换前后对比（手动验证）:")
for i in range(min(3, N)):
    p_before = pts[i, :]
    p_after = pts_transformed[i, :]
    # 手动计算预期：p_before @ Rz.T + t
    p_expected = p_before @ Rz.T + t
    diff = np.linalg.norm(p_after - p_expected)
    print(f"  点 {i}: 变换后 = {p_after}, 预期 = {p_expected}, 误差 = {diff:.2e}")
