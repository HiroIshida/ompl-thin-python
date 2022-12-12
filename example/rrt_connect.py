import _omplpy
import numpy as np
import time
#import matplotlib.pyplot as plt

def is_valid(x):
    time.sleep(0.01)
    return np.linalg.norm(np.array(x) - np.array([0.5, 0.5])) > 0.4


#planner = _omplpy.OMPLPlanner([0, 0], [1, 1], is_valid, 1000, 0.05)
#ret = planner.solve([0.1, 0.1], [0.9, 0.9])

lightning = _omplpy.LightningPlanner([0, 0], [1, 1], is_valid, 100000, 0.05, True)
lightning.solve([0.1, 0.1], [0.9, 0.9])
paths = lightning.get_experienced_paths()
print(paths)
lightning.solve([0.12, 0.04], [0.93, 0.91])
paths = lightning.get_experienced_paths()
print(paths)

lightning.recall()
lightning.solve([0.1, 0.1], [0.9, 0.9])

#setup = _omplpy.LightningSetup([0, 0], [1, 1], lambda x: True, 1000)
#_omplpy.set_algorithm_lightning(setup, "rrtconnect")
#_omplpy.solve_lightning(setup, [0.1, 0.1], [0.9, 0.9])

#planner = _omplpy.OMPLPlanner([0, 0], [1, 1], is_valid, 1000)
#points = planner.solve([0.1, 0.1], [0.9, 0.9])
#
#for i in range(len(points) - 1):
#    p0 = points[i]
#    p1 = points[i + 1]
#    plt.plot([p0[0], p1[0]], [p0[1], p1[1]])
#plt.show()
