import _omplpy
#import matplotlib.pyplot as plt
setup = _omplpy.SimpleSetup([0, 0], [1, 1], lambda x: True, 1000)
_omplpy.set_algorithm(setup, "rrtconnect")
_omplpy.solve(setup, [0.1, 0.1], [0.9, 0.9])

#planner = _omplpy.OMPLPlanner([0, 0], [1, 1], is_valid, 1000)
#points = planner.solve([0.1, 0.1], [0.9, 0.9])
#
#for i in range(len(points) - 1):
#    p0 = points[i]
#    p1 = points[i + 1]
#    plt.plot([p0[0], p1[0]], [p0[1], p1[1]])
#plt.show()
