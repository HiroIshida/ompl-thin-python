import _omplpy
planner = _omplpy.OMPLPlanner([0, 0], [1, 1], lambda x: True, 1000)
vecs = planner.solve([0.1, 0.1], [0.9, 0.9])
print(vecs)
