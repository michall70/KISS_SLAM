from kiss_icp.pybind import kiss_icp_pybind
from functions import exp_map
import matplotlib.pyplot as plt



# initial_threshold = 100

# estimator = kiss_icp_pybind._AdaptiveThreshold(
#     initial_threshold=initial_threshold,
#     min_motion_th=0.1,
#     max_range=0.5,
# )

# threshold = []
# N = 100
# for i in range(N):
#     xi = [1, 0, 0, 0, 0, 0]
#     # xi = [0, 0, 0, (i + 1) * 0.01, 0, 0]
#     # xi = [(i + 1) * 0.1, 0, 0, 0, 0, 0]
#     model_deviation = exp_map(xi)
#     estimator._update_model_deviation(model_deviation=model_deviation)
#     threshold.append(estimator._compute_threshold())



from threshold import Estimator

initial_threshold = 100
estimator = Estimator(100, 0.4, initial_threshold)

threshold = []
N = 100
for i in range(N):
    xi = [1, 0, 0, 0, 0, 0]
    # xi = [0, 0, 0, (i + 1) * 0.01, 0, 0]
    # xi = [(i + 1) * 0.1, 0, 0, 0, 0, 0]
    model_deviation = exp_map(xi)
    threshold.append(estimator.compute_threshold(model_deviation))



plt.figure(figsize=(8, 5))
# 明确 x 轴为 i（即 0 到 N-1），y 轴为 threshold
plt.plot(range(N), threshold, marker='o')
plt.xlabel('i')               # x 轴标签
plt.ylabel('threshold')       # y 轴标签
plt.title('Threshold over iterations')
plt.yscale('linear')
plt.grid(True)
print(initial_threshold)
plt.show()