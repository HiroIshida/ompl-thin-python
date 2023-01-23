from abc import ABC, abstractmethod
from enum import Enum
from pathlib import Path
from typing import Any, Callable, List, Optional, Sequence, Tuple, Union

import numpy as np

from . import _omplpy

VectorLike = Union[np.ndarray, Sequence[float]]
IsValidFunc = Callable[[List[float]], bool]


class Algorithm(Enum):
    BKPIECE1 = "BKPIECE1"
    KPIECE1 = "KPIECE1"
    LBKPIECE1 = "LBKPIECE1"
    RRTConnect = "RRTConnect"
    RRT = "RRT"
    RRTstar = "RRTstar"


def set_ompl_random_seed(seed: int) -> None:
    _omplpy.set_random_seed(seed)


class _OMPLPlannerBase(ABC):
    """
    An additional higher layer wrapper.
    The primal reason of this class is exposing type hinting
    """

    _planner: Any
    _is_valid: IsValidFunc

    def __init__(
        self,
        lb: VectorLike,
        ub: VectorLike,
        is_valid: IsValidFunc,
        n_max_is_valid: int,
        validation_box: Union[np.ndarray, float],
        algo: Algorithm = Algorithm.RRTConnect,
        algo_range: Optional[float] = None,
    ):

        lb = np.array(lb)
        ub = np.array(ub)
        assert lb.ndim == 1
        assert ub.ndim == 1
        assert len(lb) == len(ub)
        planner_t = self.planner_type()
        if isinstance(validation_box, float):
            dim = len(lb)
            validation_box = np.array([validation_box] * dim)
        self._planner = planner_t(
            lb, ub, is_valid, n_max_is_valid, validation_box, algo.value
        )
        self.reset_is_valid(is_valid)

    def solve(
        self, start: VectorLike, goal: VectorLike, simplify: bool = False
    ) -> Optional[List[np.ndarray]]:
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


class _UnconstrainedPlannerBase(_OMPLPlannerBase):
    def __init__(
        self,
        lb: VectorLike,
        ub: VectorLike,
        is_valid: IsValidFunc,
        n_max_is_valid: int,
        validation_box: Union[np.ndarray, float],
        algo: Algorithm = Algorithm.RRTConnect,
        algo_range: Optional[float] = None,
    ):

        lb = np.array(lb)
        ub = np.array(ub)
        assert lb.ndim == 1
        assert ub.ndim == 1
        assert len(lb) == len(ub)
        planner_t = self.planner_type()
        if isinstance(validation_box, float):
            dim = len(lb)
            validation_box = np.array([validation_box] * dim)
        self._planner = planner_t(
            lb, ub, is_valid, n_max_is_valid, validation_box, algo.value, algo_range
        )
        self.reset_is_valid(is_valid)


class Planner(_UnconstrainedPlannerBase):
    def planner_type(self) -> Any:
        return _omplpy._OMPLPlanner


class RepairPlanner(_UnconstrainedPlannerBase):
    def planner_type(self) -> Any:
        return _omplpy._LightningRepairPlanner

    def set_heuristic(self, traj_heuristic: np.ndarray) -> None:
        self._planner.set_heuristic(traj_heuristic)


class ConstrainedPlanner(_OMPLPlannerBase):
    def __init__(
        self,
        eq_const: Callable[[np.ndarray], Tuple[np.ndarray, np.ndarray]],
        lb: VectorLike,
        ub: VectorLike,
        is_valid: IsValidFunc,
        n_max_is_valid: int,
        validation_box: Union[np.ndarray, float],
        algo: Algorithm = Algorithm.RRTConnect,
        algo_range: Optional[float] = None,
    ):

        lb = np.array(lb)
        ub = np.array(ub)
        assert lb.ndim == 1
        assert ub.ndim == 1
        assert len(lb) == len(ub)
        planner_t = self.planner_type()
        if isinstance(validation_box, float):
            dim = len(lb)
            validation_box = np.array([validation_box] * dim)

        class ConstraintFunction:
            jac_cache: Optional[np.ndarray] = None

            def __call__(self, x: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
                return eq_const(x)

            def f(self, x: np.ndarray) -> List[float]:
                val, jac = self.__call__(x)
                self.jac_cache = jac
                return val.tolist()

            def jac(self, x: np.ndarray) -> List[List[float]]:
                assert self.jac_cache is not None
                return self.jac_cache.tolist()

        const_fn = ConstraintFunction()
        self._planner = planner_t(
            const_fn.f,
            const_fn.jac,
            lb,
            ub,
            is_valid,
            n_max_is_valid,
            validation_box,
            algo.value,
            algo_range,
        )
        self.reset_is_valid(is_valid)

    def solve(
        self, start: VectorLike, goal: VectorLike, simplify: bool = False
    ) -> Optional[List[np.ndarray]]:

        # unlike unconstrained case, due to appliximation nature interpolation,
        # start and goal can be deviated from true ones. Thus, we will apend them
        np_start = np.array(start)
        np_goal = np.array(goal)
        traj = super().solve(start, goal, simplify=simplify)
        if traj is None:
            return None
        if np.linalg.norm(traj[0] - np_start) > 1e-5:
            traj[0] = np_start
        if np.linalg.norm(traj[-1] - np_goal) > 1e-5:
            traj[-1] = np_goal
        return traj

    def planner_type(self) -> Any:
        return _omplpy._ConstrainedPlanner


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
    def __init__(
        self,
        db: LightningDB,
        lb: VectorLike,
        ub: VectorLike,
        is_valid: IsValidFunc,
        n_max_is_valid: int,
        validation_box: Union[np.ndarray, float],
        algo: Algorithm = Algorithm.RRTConnect,
        algo_range: Optional[float] = None,
    ):

        lb = np.array(lb)
        ub = np.array(ub)
        assert lb.ndim == 1
        assert ub.ndim == 1
        assert len(lb) == len(ub)
        planner_t = self.planner_type()
        if isinstance(validation_box, float):
            dim = len(lb)
            validation_box = np.array([validation_box] * dim)
        self._planner = planner_t(
            db, lb, ub, is_valid, n_max_is_valid, validation_box, algo.value, algo_range
        )
        self.reset_is_valid(is_valid)

    def planner_type(self) -> Any:
        return _omplpy._LightningPlanner
