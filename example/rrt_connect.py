import _omplpy
import matplotlib.pyplot as plt
import numpy as np

def is_valid(vec):
    return np.linalg.norm(np.array(vec) - np.ones(2) * 0.5) > 0.3


planner = _omplpy.OMPLPlanner([0, 0], [1, 1], is_valid, 1000)
points = planner.solve([0.1, 0.1], [0.9, 0.9])

for i in range(len(points) - 1):
    p0 = points[i]
    p1 = points[i + 1]
    plt.plot([p0[0], p1[0]], [p0[1], p1[1]])
plt.show()
