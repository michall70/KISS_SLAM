import numpy as np

a = np.arange(10)
print(a.dtype)

b = np.ones((3,4))
print(b)
print(b.dtype)

c = [3, 4]
print(np.linalg.norm(c))