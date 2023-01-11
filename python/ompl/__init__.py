from abc import ABC, abstractmethod
from enum import Enum
from typing import Sequence, Any, Callable, Optional, List, Union
import numpy as np
from pathlib import Path

from . import _omplpy

VectorLike = Union[np.ndarray, Sequence[float]]
IsValidFunc = Callable[[List[float]], bool]


class Algorithm(Enum):
    # informed
    ABITstar = "ABITstar"
    AITstar = "AITstar"
    BITstar = "BITstar"
    # kpiece
    BKPIECE1 = "BKPIECE1"
    KPIECE1 = "KPIECE1"
    LBKPIECE1 = "LBKPIECE1"
    # rrt
    RRTConnect = "RRTConnect"
    RRT = "RRT"
    RRTstar = "RRTstar"
    SORRTstar = "SORRTstar"
    RRTsharp = "RRTsharp"
    InformedRRTstar = "InformedRRTstar"
    # prm
    PRMstar = "PRMstar"
    LazyPRMstar = "LazyPRMstar"
    LazyPRM = "LazyPRM"
    PRM = "PRM"
    # fmt
    BFMT = "BFMT"
    FMT = "FMT"


def set_ompl_random_seed(seed: int) -> None:
    _omplpy.set_random_seed(seed)


class _OMPLPlannerBase(ABC):
    """
    An additional higher layer wrapper.
    The primal reason of this class is exposing type hinting
    """
    _planner: Any
    _is_valid: IsValidFunc

    def __init__(self,
            lb: VectorLike,
            ub: VectorLike,
            is_valid: IsValidFunc,
            n_max_is_valid: int,
            validation_box: Union[np.ndarray, float],
            algo: Algorithm = Algorithm.RRTConnect):

        lb = np.array(lb)
        ub = np.array(ub)
        assert lb.ndim == 1
        assert ub.ndim == 1
        assert len(lb) == len(ub)
        planner_t = self.planner_type()
        if isinstance(validation_box, float):
            dim = len(lb)
            validation_box = np.array([validation_box] * dim)
        self._planner = planner_t(lb, ub, is_valid, n_max_is_valid, validation_box, algo.value)
        self.reset_is_valid(is_valid)

    def solve(self, start: VectorLike, goal: VectorLike, simplify: bool=False) -> Optional[List[np.ndarray]]:
        start = np.array(start)
        goal = np.array(goal)

        assert self._is_valid(start.tolist())
        assert self._is_valid(goal.tolist())
        ret = self._planner.solve(start, goal, simplify)
        if ret is None:
            return ret
        for i in range(len(ret)):
            ret[i] = np.array(ret[i])
        return ret

    def reset_is_valid(self, is_valid: IsValidFunc) -> None:
        self._planner.reset_is_valid(is_valid)
        self._is_valid = is_valid

    @abstractmethod
    def planner_type(self) -> Any:
        ...


class Planner(_OMPLPlannerBase):

    def __init__(self,
            lb: VectorLike,
            ub: VectorLike,
            is_valid: IsValidFunc,
            n_max_is_valid: int,
            validation_box: Union[np.ndarray, float],
            algo: Algorithm = Algorithm.RRTConnect):

        lb = np.array(lb)
        ub = np.array(ub)
        assert lb.ndim == 1
        assert ub.ndim == 1
        assert len(lb) == len(ub)
        planner_t = self.planner_type()
        if isinstance(validation_box, float):
            dim = len(lb)
            validation_box = np.array([validation_box] * dim)
        self._planner = planner_t(lb, ub, is_valid, n_max_is_valid, validation_box, algo.value)
        self.reset_is_valid(is_valid)

    def planner_type(self) -> Any:
        return _omplpy._OMPLPlanner


class LightningDB(_omplpy._LightningDB):
    dim: int

    def __init__(self, dim: int):
        self.dim = dim
        super().__init__(dim)

    def add_experience(self, experience: List[np.ndarray]) -> None:
        assert experience[0].shape == (self.dim,)
        super().add_experience(np.array(experience).tolist())

    def get_experienced_paths(self) -> List[np.ndarray]:
        paths = super().get_experienced_paths()
        return [np.array(p) for p in paths]

    def get_experiences_count(self) -> int:
        return super().get_experiences_count()

    def save(self, path: Union[Path, str]) -> None:
        if isinstance(path, Path):
            path = str(path.expanduser())
        super().save(path)

    def load(self, path: Union[Path, str]) -> None:
        if isinstance(path, Path):
            path = str(path.expanduser())
        super().load(path)


class LightningPlanner(_OMPLPlannerBase):

    def __init__(self,
            db: LightningDB,
            lb: VectorLike,
            ub: VectorLike,
            is_valid: IsValidFunc,
            n_max_is_valid: int,
            validation_box: Union[np.ndarray, float],
            algo: Algorithm = Algorithm.RRTConnect):

        lb = np.array(lb)
        ub = np.array(ub)
        assert lb.ndim == 1
        assert ub.ndim == 1
        assert len(lb) == len(ub)
        planner_t = self.planner_type()
        if isinstance(validation_box, float):
            dim = len(lb)
            validation_box = np.array([validation_box] * dim)
        self._planner = planner_t(db, lb, ub, is_valid, n_max_is_valid, validation_box, algo.value)
        self.reset_is_valid(is_valid)

    def planner_type(self) -> Any:
        return _omplpy._LightningPlanner


class PathSimplifier(_omplpy._PathSimplifier):

    def __init__(self,
            lb: VectorLike,
            ub: VectorLike,
            is_valid: IsValidFunc,
            n_max_is_valid: int,
            validation_box: Union[np.ndarray, float]):

        lb = np.array(lb)
        ub = np.array(ub)
        assert lb.ndim == 1
        assert ub.ndim == 1
        assert len(lb) == len(ub)
        if isinstance(validation_box, float):
            dim = len(lb)
            validation_box = np.array([validation_box] * dim)
        super().__init__(lb, ub, is_valid, n_max_is_valid, validation_box)

    def simplify(self, ps: List[np.ndarray]) -> List[np.ndarray]:
        lst_lst = super().simplify(ps)
        return [np.array(lst) for lst in lst_lst]
