"""校验整组目标，再按规范顺序转发。"""

from math import isfinite

from .config import JOINT_NAMES, LEGS
from .interfaces import Backend, FootTargets, JointTargets, RobotObservation


class Controller:
    def __init__(self, backend: Backend):
        self.backend = backend

    def read(self) -> RobotObservation:
        observation = self.backend.read()
        for values, count in (
            (observation.position, len(JOINT_NAMES)),
            (observation.velocity, len(JOINT_NAMES)),
            (observation.angular_velocity, 3),
            (observation.projected_gravity, 3),
        ):
            if len(values) != count or not all(map(isfinite, values)):
                raise ValueError("Backend observation has an invalid shape or value")
        return observation

    def submit(self, targets: JointTargets | FootTargets) -> None:
        if isinstance(targets, JointTargets):
            self.backend.write_joints(normalize_joint_targets(targets))
        elif isinstance(targets, FootTargets):
            if len(targets.positions) != len(LEGS) or any(
                len(position) != 3 or not all(map(isfinite, position))
                for position in targets.positions
            ):
                raise ValueError("Foot targets must contain four finite 3D positions")
            values = _ordered(LEGS, targets.legs, targets.positions)
            self.backend.write_feet(FootTargets(LEGS, values))
        else:
            raise TypeError("Unsupported target type")


def normalize_joint_targets(targets: JointTargets) -> JointTargets:
    if len(targets.position) != len(JOINT_NAMES) or not all(map(isfinite, targets.position)):
        raise ValueError("Joint targets must contain 12 finite angles")
    values = _ordered(JOINT_NAMES, targets.names, targets.position)
    return JointTargets(JOINT_NAMES, values)


def _ordered(expected, names, values):
    if len(names) != len(expected) or set(names) != set(expected):
        raise ValueError("Target names are missing or duplicated")
    entries = dict(zip(names, values))
    return tuple(entries[name] for name in expected)
