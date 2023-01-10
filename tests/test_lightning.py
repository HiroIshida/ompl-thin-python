import time
import numpy as np
import matplotlib.pyplot as plt

from ompl import LightningDB, LightningPlanner, Planner


def test_lightning(visualize: bool = False):

    def is_valid(p):
        # create difficult problem
        time.sleep(1e-5)
        x, y = p
        if x < 0.2 or x > 0.8:
            return True
        if abs(y - 0.5) < 0.006:
            return True
        return False

    planner = Planner([0, 0], [1, 1], is_valid, 1000000, 0.001)
    ts = time.time()
    res = planner.solve([0.1, 0.1], [0.9, 0.9])
    elapsed_rrtconnect = time.time() - ts
    assert res is not None

    db = LightningDB(2)
    db.add_experience(res)
    lighting = LightningPlanner(db, [0, 0], [1, 1], is_valid, 100000, 0.001)
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
