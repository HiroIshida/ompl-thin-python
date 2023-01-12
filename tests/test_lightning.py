import time

import matplotlib.pyplot as plt
import numpy as np
from common import IsValid, ProblemDef

from ompl import LightningDB, LightningPlanner, Planner


def test_lightning(visualize: bool = False):

    is_valid = IsValid(1e-5)

    pdef = ProblemDef()

    planner = Planner(pdef.lb, pdef.ub, is_valid, pdef.n_max_call, pdef.motion_step_box)
    ts = time.time()
    res = planner.solve(pdef.start, pdef.goal)
    elapsed_rrtconnect = time.time() - ts
    assert res is not None

    db = LightningDB(2)
    db.add_experience(res)
    lighting = LightningPlanner(
        db, pdef.lb, pdef.ub, is_valid, pdef.n_max_call, pdef.motion_step_box
    )
    ts = time.time()
    lightning_result = lighting.solve([0.1, 0.2], [0.9, 0.8])
    elapsed_lightning = time.time() - ts

    assert elapsed_lightning < 0.2 * elapsed_rrtconnect

    if visualize:
        traj = np.array(lightning_result)
        plt.plot(traj[:, 0], traj[:, 1])
        plt.show()


if __name__ == "__main__":
    test_lightning(True)
