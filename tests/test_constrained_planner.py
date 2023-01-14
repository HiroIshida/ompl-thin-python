from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D, axes3d  # noqa

from ompl import ConstrainedPlanner, set_ompl_random_seed

set_ompl_random_seed(0)
np.random.seed(0)


def plt_sphere(list_center, list_radius, fig, ax):
    for c, r in zip(list_center, list_radius):

        # draw sphere
        u, v = np.mgrid[0 : 2 * np.pi : 50j, 0 : np.pi : 50j]
        x = r * np.cos(u) * np.sin(v)
        y = r * np.sin(u) * np.sin(v)
        z = r * np.cos(v)

        ax.plot_surface(
            x - c[0],
            y - c[1],
            z - c[2],
            color=np.random.choice(["g", "b"]),
            alpha=0.5 * np.random.random() + 0.5,
        )


def eq_const(x: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    y = np.array([np.linalg.norm(x) - 1])

    dist = np.linalg.norm(x)
    jac = np.expand_dims((x / dist), axis=0)
    return y, jac


def is_valid(vec: List[float]) -> bool:
    x, y, z = vec
    if abs(x) < 0.2 and abs(z) < 0.8:
        return False
    return True


def test_constrained_planner(visualize: bool = False):
    planner = ConstrainedPlanner(
        eq_const, [-2, -2, -2], [2, 2, 2], is_valid, 10000, 0.1, algo_range=0.3
    )
    start = np.array([-1, 0.0, 0.0])
    goal = np.array([1, 0.0, 0.0])
    traj = planner.solve(start, goal, False)

    assert traj is not None
    for p in traj:
        val = eq_const(p)[0]
        assert np.all(val < 1e-3)
    assert np.linalg.norm(traj[0] - start) < 1e-3
    assert np.linalg.norm(traj[-1] - goal) < 1e-3

    if visualize:
        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")
        plt_sphere([[0, 0, 0]], [0.97], fig, ax)
        X = np.array(traj)
        ax.scatter(X[:, 0], X[:, 1], X[:, 2])
        plt.show()


if __name__ == "__main__":
    test_constrained_planner(True)
