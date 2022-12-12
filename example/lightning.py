import time

import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

from ompl import LightningPlanner


def is_valid(x):
    if np.linalg.norm(np.array(x) - np.array([0.25, 0.5])) < 0.25:
        return False
    if np.linalg.norm(np.array(x) - np.array([1.0, 0.0])) < 0.4:
        return False
    return True


def sample_valid():
    while True:
        x = np.random.rand(2)
        if is_valid(x):
            return x


def plot_trajectory(ax, points, color):
    points = np.array(points)
    for i in range(len(points) - 1):
        p0 = points[i]
        p1 = points[i + 1]
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color=color)
    ax.scatter(points[:, 0], points[:, 1], c="black")


if __name__ == "__main__":
    # store experience
    lightning = LightningPlanner([0, 0], [1, 1], is_valid, 100000, 0.05)
    lightning.scratch()
    for _ in range(300):
        lightning.solve(sample_valid(), sample_valid())
    lightning.dump("tmp.db")

    # use experience to solve faster
    lightning_loaded = LightningPlanner([0, 0], [1, 1], is_valid, 100000, 0.05)
    lightning_loaded.load("tmp.db")
    lightning_loaded.recall()
    start = np.array([0.1, 0.1])
    goal = np.array([0.9, 0.9])
    lightning_loaded.solve(start, goal)

    # visualization
    paths = lightning_loaded.get_experienced_paths()
    fig, ax = plt.subplots()
    for path in paths:
        plot_trajectory(ax, path, "red")
    idx = lightning_loaded.get_latest_activated_index()
    plot_trajectory(ax, paths[idx], "blue")
    plt.show()
