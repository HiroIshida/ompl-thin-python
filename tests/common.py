import time
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


@dataclass
class IsValid:
    time_pause: float = 0

    def __call__(self, p):
        # create difficult problem
        time.sleep(self.time_pause)
        x, y = p
        if x < 0.2 or x > 0.8:
            return True
        if abs(y - 0.5) < 0.006:
            return True
        return False


@dataclass
class ProblemDef:
    lb: np.ndarray = np.zeros(2)
    ub: np.ndarray = np.ones(2)
    n_max_call: int = 100000000
    motion_step_box: float = 0.001
    start: np.ndarray = np.ones(2) * 0.1
    goal: np.ndarray = np.ones(2) * 0.9


@dataclass
class Trajectory:
    """utility class to handle trajectory"""

    points: List[np.ndarray]

    @property
    def length(self) -> float:
        n_point = len(self.points)
        total = 0.0
        for i in range(n_point - 1):
            p0 = self.points[i]
            p1 = self.points[i + 1]
            total += float(np.linalg.norm(p1 - p0))
        return total

    def visualize(self, fax: Tuple, *args, **kwargs) -> None:
        fig, ax = fax
        arr = np.array(self.points)
        ax.plot(arr[:, 0], arr[:, 1], *args, **kwargs)

    def sample_point(self, dist_from_start: float) -> np.ndarray:

        if dist_from_start > self.length + 1e-6:
            assert False

        dist_from_start = min(dist_from_start, self.length)
        edge_dist_sum = 0.0
        for i in range(len(self.points) - 1):
            edge_dist_sum += float(np.linalg.norm(self.points[i + 1] - self.points[i]))
            if dist_from_start <= edge_dist_sum:
                diff = edge_dist_sum - dist_from_start
                vec_to_prev = self.points[i] - self.points[i + 1]
                vec_to_prev_unit = vec_to_prev / np.linalg.norm(vec_to_prev)
                point_new = self.points[i + 1] + vec_to_prev_unit * diff
                return point_new
        assert False

    def resample(self, n_waypoint: int) -> "Trajectory":
        # yeah, it's inefficient. n^2 instead of n ...
        point_new_list = []
        partial_length = self.length / (n_waypoint - 1)
        for i in range(n_waypoint):
            dist_from_start = partial_length * i
            point_new = self.sample_point(dist_from_start)
            point_new_list.append(point_new)
        return Trajectory(point_new_list)
