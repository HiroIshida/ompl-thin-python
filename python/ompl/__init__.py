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
    BITstar = "BITstar"
    BITstarStop = "BITstarStop"  # stop after first solution


class ConstStateType(Enum):
    PROJECTION = _omplpy.ConstStateType.PROJECTION
    ATLAS = _omplpy.ConstStateType.ATLAS
    TANGENT = _omplpy.ConstStateType.TANGENT


def set_ompl_random_seed(seed: int) -> None:
    _omplpy.set_random_seed(seed)


def turn_off_logger() -> None:
    _omplpy.set_log_level_none()


class InvalidProblemError(Exception):
    ...


class _OMPLPlannerBase(ABC):
    """
    An additional higher layer wrapper.
    The primal reason of this class is exposing type hinting
    """

    _planner: Any
    _is_valid: IsValidFunc
    _lb: np.ndarray
    _ub: np.ndarray

    def solve(
        self, start: VectorLike, goal: VectorLike, simplify: bool = False
    ) -> Optional[List[np.ndarray]]:
        start = np.array(start)
        goal = np.array(goal)

        # sanity check
        if not self._is_valid(start.tolist()):
            raise InvalidProblemError("start position is not valid")

        if np.any(start < np.array(self._lb)) or np.any(start > np.array(self._ub)):
            raise InvalidProblemError("start position is not in box")

        if not self._is_valid(goal.tolist()):
            raise InvalidProblemError("goal position is not valid")

        if np.any(goal < np.array(self._lb)) or np.any(goal > np.array(self._ub)):
            raise InvalidProblemError("gol position is not in box")

        ret = self._planner.solve(start, goal, simplify)
        if ret is None:
            return ret
        for i in range(len(ret)):
            ret[i] = np.array(ret[i])
        return ret

    def reset_is_valid(self, is_valid: IsValidFunc) -> None:
        self._planner.reset_is_valid(is_valid)
        self._is_valid = is_valid


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
        if isinstance(validation_box, float):
            dim = len(lb)
            validation_box = np.array([validation_box] * dim)

        self._planner = self._create_planner(
            lb, ub, is_valid, n_max_is_valid, validation_box, algo, algo_range
        )
        self.reset_is_valid(is_valid)
        self._lb = lb
        self._ub = ub

    @abstractmethod
    def _create_planner(
        self,
        lb: VectorLike,
        ub: VectorLike,
        is_valid: IsValidFunc,
        n_max_is_valid: int,
        validation_box: np.ndarray,
        algo: Algorithm = Algorithm.RRTConnect,
        algo_range: Optional[float] = None,
    ) -> Any:
        ...


class Planner(_UnconstrainedPlannerBase):
    def _create_planner(
        self,
        lb: VectorLike,
        ub: VectorLike,
        is_valid: IsValidFunc,
        n_max_is_valid: int,
        validation_box: np.ndarray,
        algo: Algorithm = Algorithm.RRTConnect,
        algo_range: Optional[float] = None,
    ) -> Any:
        return _omplpy._OMPLPlanner(
            lb, ub, is_valid, n_max_is_valid, validation_box, algo.value, algo_range
        )


class RepairPlanner(_UnconstrainedPlannerBase):
    def _create_planner(
        self,
        lb: VectorLike,
        ub: VectorLike,
        is_valid: IsValidFunc,
        n_max_is_valid: int,
        validation_box: np.ndarray,
        algo: Algorithm = Algorithm.RRTConnect,
        algo_range: Optional[float] = None,
    ) -> Any:
        return _omplpy._LightningRepairPlanner(
            lb, ub, is_valid, n_max_is_valid, validation_box, algo.value, algo_range
        )

    def set_heuristic(self, traj_heuristic: np.ndarray) -> None:
        self._planner.set_heuristic(traj_heuristic)


class ERTConnectPlanner(_UnconstrainedPlannerBase):
    _is_set_heursitic: bool = False

    def _create_planner(
        self,
        lb: VectorLike,
        ub: VectorLike,
        is_valid: IsValidFunc,
        n_max_is_valid: int,
        validation_box: np.ndarray,
        algo: Algorithm = Algorithm.RRTConnect,
        algo_range: Optional[float] = None,
    ) -> Any:
        # NOTE: algo will not used in this planner
        return _omplpy._ERTConnectPlanner(
            lb, ub, is_valid, n_max_is_valid, validation_box
        )

    def set_parameters(
        self,
        omega_min: Optional[float] = None,
        omega_max: Optional[float] = None,
        eps: Optional[float] = None,
    ):
        self._planner.set_parameters(omega_min, omega_max, eps)

    def solve(
        self, start: VectorLike, goal: VectorLike, simplify: bool = False
    ) -> Optional[List[np.ndarray]]:
        if not self._is_set_heursitic:
            raise RuntimeError("trajectory heuristic is not set")
        return super().solve(start, goal, simplify)

    def set_heuristic(self, traj_heuristic: np.ndarray) -> None:
        self._planner.set_heuristic(traj_heuristic)
        self._is_set_heursitic = True


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
        if isinstance(validation_box, float):
            dim = len(lb)
            validation_box = np.array([validation_box] * dim)
        self._planner = _omplpy._LightningPlanner(
            db, lb, ub, is_valid, n_max_is_valid, validation_box, algo.value, algo_range
        )
        self.reset_is_valid(is_valid)
        self._lb = lb
        self._ub = ub


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
        cs_type: ConstStateType = ConstStateType.PROJECTION,
    ):

        lb = np.array(lb)
        ub = np.array(ub)
        assert lb.ndim == 1
        assert ub.ndim == 1
        assert len(lb) == len(ub)
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

        self._planner = _omplpy._ConstrainedPlanner(
            const_fn.f,
            const_fn.jac,
            lb,
            ub,
            is_valid,
            n_max_is_valid,
            validation_box,
            algo.value,
            algo_range,
            cs_type.value,
        )
        self.reset_is_valid(is_valid)
        self._lb = lb
        self._ub = ub

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
