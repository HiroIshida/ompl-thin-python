import matplotlib.pyplot as plt
from common import IsValid, ProblemDef, Trajectory

from ompl import Planner, set_ompl_random_seed

set_ompl_random_seed(0)


def test_planner(visualize: bool = False):

    is_valid = IsValid(0)
    pdef = ProblemDef()

    planner = Planner(pdef.lb, pdef.ub, is_valid, pdef.n_max_call, pdef.motion_step_box)
    ret = planner.solve(pdef.start, pdef.goal)
    assert ret is not None
    traj1 = Trajectory(list(ret))

    ret = planner.solve(pdef.start, pdef.goal, simplify=True)
    assert ret is not None
    traj2 = Trajectory(list(ret))

    assert traj2.length < traj1.length  # simplified path must be shorted

    for traj in [traj1, traj2]:
        n_large_enough = 100
        for p in traj.resample(n_large_enough).points:
            assert is_valid(p)

    if visualize:
        fig, ax = plt.subplots()
        traj1.visualize((fig, ax), "ro-")
        traj2.visualize((fig, ax), "go-")
        plt.show()


if __name__ == "__main__":
    test_planner(visualize=True)
