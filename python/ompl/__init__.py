from . import _omplpy
from typing import Sequence, Any, Callable, Optional, List, Union
import numpy as np
from pathlib import Path

VectorLike = Union[np.ndarray, Sequence[float]]
PathLike = Union[Path, str]


class Planner:
    """
    An additional higher layer wrapper.
    One important reason of this class is exposing type hinting
    """
    _planner: Any
    _is_valid: Callable[[VectorLike], bool]

    def __init__(self, lb: VectorLike, ub: VectorLike, is_valid: Callable[[VectorLike], bool], n_max_is_valid: int, fraction: float):
        self._planner = _omplpy._OMPLPlanner(lb, ub, is_valid, n_max_is_valid, fraction)
        self._is_valid = is_valid

    def solve(self, start: VectorLike, goal: VectorLike) -> Optional[List[np.ndarray]]:
        assert self._is_valid(start)
        assert self._is_valid(goal)
        ret = self._planner.solve(start, goal)
        if ret is None:
            return ret
        for i in range(len(ret)):
            ret[i] = np.array(ret[i])
        return ret


class LightningPlanner:
    """
    An additional higher layer wrapper.
    One important reason of this class is exposing type hinting
    """
    _planner: Any
    _is_valid: Callable[[VectorLike], bool]

    def __init__(self, lb: VectorLike, ub: VectorLike, is_valid: Callable[[VectorLike], bool], n_max_is_valid: int, fraction: float):
        self._planner = _omplpy._LightningPlanner(lb, ub, is_valid, n_max_is_valid, fraction)
        self._is_valid = is_valid

    def solve(self, start: VectorLike, goal: VectorLike) -> Optional[List[np.ndarray]]:
        assert self._is_valid(start)
        assert self._is_valid(goal)
        ret = self._planner.solve(start, goal)
        if ret is None:
            return ret
        for i in range(len(ret)):
            ret[i] = np.array(ret[i])
        return ret

    def scratch(self) -> None:
        self._planner.scratch()

    def recall(self) -> None:
        self._planner.recall()

    def dump(self, path: PathLike) -> None:
        if isinstance(path, Path):
            path = str(path.expanduser())
        self._planner.dump(path)

    def load(self, path: PathLike) -> None:
        if isinstance(path, Path):
            path = str(path.expanduser())
        self._planner.load(path)

    def get_experienced_paths(self) -> List[np.ndarray]:
        paths = self._planner.get_experienced_paths()
        return paths

    def get_latest_activated_index(self) -> int:
        return self._planner.get_latest_activated_index()
