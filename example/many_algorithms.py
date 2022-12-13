from ompl import Planner, Algorithm
import numpy as np
import time


def is_valid(x) -> bool:
    r = np.linalg.norm(np.array(x) - np.ones(2) * 0.5)
    return bool(r > 0.35)

start = np.array([0.1, 0.1])
goal = np.array([0.9, 0.9])

skips = ["AITstar"]

for algo in Algorithm:
    print(algo)
    if algo.value in skips:
        continue
    planner = Planner([0, 0], [1, 1], is_valid, 1000, 0.05, algo)
    ret = planner.solve(start, goal)
