import matplotlib.pyplot as plt
import numpy as np
import pytest
from common import IsValid, ProblemDef, Trajectory

from ompl import ERTConnectPlanner, Planner, RepairPlanner, set_ompl_random_seed

set_ompl_random_seed(0)


def _test_heuristic_planner(heuristic_planner_t, visualize: bool = False):

    is_valid = IsValid(0)
    pdef = ProblemDef()

    repair_planner = heuristic_planner_t(
        pdef.lb, pdef.ub, is_valid, pdef.n_max_call, pdef.motion_step_box
    )
    planner = Planner(pdef.lb, pdef.ub, is_valid, pdef.n_max_call, pdef.motion_step_box)

    if isinstance(repair_planner, ERTConnectPlanner):
        repair_planner.set_parameters(eps=0.1)

    with pytest.raises(RuntimeError):
        # because we didnt set heuristic yet
        ret = repair_planner.solve(pdef.start, pdef.goal, simplify=True)

    ret = planner.solve(pdef.start, pdef.goal)
    assert ret is not None

    repair_planner.set_heuristic(ret)
    ret = repair_planner.solve(
        pdef.start + np.random.rand(2) * 0.01,
        pdef.goal + np.random.rand(2) * 0.01,
        simplify=True,
    )
    traj = Trajectory(ret)

    n_large_enough = 100
    for p in traj.resample(n_large_enough).points:
        assert is_valid(p)

    if visualize:
        fig, ax = plt.subplots()
        traj.visualize((fig, ax), "ro-")
        plt.show()


def test_heuristic_planner():
    _test_heuristic_planner(RepairPlanner)
    _test_heuristic_planner(ERTConnectPlanner)


if __name__ == "__main__":
    _test_heuristic_planner(ERTConnectPlanner, visualize=True)
