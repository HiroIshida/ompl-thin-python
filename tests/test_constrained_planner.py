import time
from typing import List

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D, axes3d  # <-- Note the capitalization!

from ompl import ConstrainedPlanner, set_ompl_random_seed

set_ompl_random_seed(0)
np.random.seed(0)


def f(x: np.ndarray) -> np.ndarray:
    time.sleep(0.001)
    y = np.array([np.linalg.norm(x) - 1])
    return y


def jac(x: np.ndarray) -> List[List[float]]:
    dist = np.linalg.norm(x)
    out = [(x / dist).tolist()]
    return out


planner = ConstrainedPlanner(
    f, jac, [-2, -2, -2], [2, 2, 2], lambda x: True, 10000, 0.1
)
ret = planner.solve([-1, 0.0, 0.0], [1, 0.0, 0.0], False)

fig = plt.figure()
ax = fig.add_subplot(projection="3d")
X = np.array(ret)
ax.scatter(X[:, 0], X[:, 1], X[:, 2])
plt.show()
