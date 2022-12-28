from ompl import Planner, Algorithm, set_random_seed
import numpy as np
import time


def is_valid(x) -> bool:
    r = np.linalg.norm(np.array(x) - np.ones(2) * 0.5)
    return bool(r > 0.35)

start = np.array([0.1, 0.1])
goal = np.array([0.9, 0.9])

skips = ["AITstar", "LazyPRMstar"]
set_random_seed(0)

for algo in Algorithm:
    print(algo)
    if algo.value in skips:
        continue
    planner = Planner([0, 0], [1, 1], is_valid, 10000, 0.05, algo)
    trajectory = planner.solve(start, goal)
    assert trajectory is not None
    if trajectory is not None:
        for p in trajectory:
            assert is_valid(p)
