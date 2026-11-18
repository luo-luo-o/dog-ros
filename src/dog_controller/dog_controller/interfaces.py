"""控制层与后端之间的同步接口。"""

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class RobotObservation:
    """按规范关节顺序排列。角速度和投影重力使用机体坐标系。"""

    position: tuple[float, ...]
    velocity: tuple[float, ...]
    angular_velocity: tuple[float, float, float]
    projected_gravity: tuple[float, float, float]


@dataclass(frozen=True)
class JointTargets:
    """按名称给出一组完整关节目标，单位弧度。"""

    names: tuple[str, ...]
    position: tuple[float, ...]


@dataclass(frozen=True)
class FootTargets:
    """按腿名给出四足位置，使用 trunk 坐标系，单位米。"""

    legs: tuple[str, ...]
    positions: tuple[tuple[float, float, float], ...]


class Backend(Protocol):
    """后端实现支持的写入类型及真实观测。"""

    def read(self) -> RobotObservation: ...

    def write_joints(self, targets: JointTargets) -> None: ...

    def write_feet(self, targets: FootTargets) -> None: ...
