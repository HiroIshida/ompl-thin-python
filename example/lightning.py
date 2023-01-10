import argparse
import time

import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

from ompl import LightningPlanner, Algorithm, Planner, LightningDB, set_ompl_random_seed, PathSimplifier

set_ompl_random_seed(0)
np.random.seed(5)


def is_valid(x):
    if np.linalg.norm(np.array(x) - np.array([0.5, 0.5])) < 0.45:
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
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color=color, lw=0.5)
    ax.scatter(points[:, 0], points[:, 1], c="black", s=0.3)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--visualize", action="store_true", help="visualize")
    args = parser.parse_args()
    visualize: bool = args.visualize

    planner = Planner([0, 0], [1, 1], is_valid, 1000, [0.04, 0.04], Algorithm.RRTConnect)

    db = LightningDB(2)
    for _ in range(30):
        trajectory = planner.solve(sample_valid(), sample_valid())
        db.add_experience(np.array(trajectory))
        print("db count: {}".format(db.get_experiences_count()))
    db.save("tmp.db")
    db_again = LightningDB(2)
    db_again.load("tmp.db")
    lightning = LightningPlanner(db_again, [0, 0], [1, 1], is_valid, 1000, [0.04, 0.04], Algorithm.RRTConnect)

    ts = time.time()
    lightning_path = lightning.solve([0.01, 0.01], [0.99, 0.99])
    print("lightning elapsed: {}".format(time.time() - ts))

    assert np.linalg.norm(lightning_path[0] - lightning_path[1]) > 1e-5
    assert np.linalg.norm(lightning_path[-1] - lightning_path[-2]) > 1e-5

    print("start simplify")
    simplifier = PathSimplifier([0, 0], [1, 1], is_valid, 1000, [0.04, 0.04])
    simplified_path = simplifier.simplify(lightning_path)

    # visualization
    if visualize:
        paths = db.get_experienced_paths()
        fig, ax = plt.subplots()
        for path in paths:
            plot_trajectory(ax, path, "red")
        plot_trajectory(ax, np.array(lightning_path), "blue")
        plot_trajectory(ax, np.array(simplified_path), "green")

        circle = plt.Circle((0.5, 0.5), 0.45, color='k', fill=False)
        ax.add_patch(circle)

        plt.show()
