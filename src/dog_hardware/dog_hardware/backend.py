"""实物后端的 IK 与舵机边界。"""

from typing import Protocol

from dog_controller.controller import normalize_joint_targets
from dog_controller.interfaces import FootTargets, JointTargets, RobotObservation


class InverseKinematics(Protocol):
    """将机体坐标系足端目标转换成模型关节角。"""

    def solve(self, targets: FootTargets) -> JointTargets: ...


class ServoDriver(Protocol):
    """将标准关节目标和观测接到待定舵机及 IMU 协议。"""

    def read(self) -> RobotObservation: ...

    def write(self, targets: JointTargets) -> None: ...


class HardwareBackend:
    """未配置 IK 及驱动时停留在占位状态。"""

    def __init__(self, ik: InverseKinematics | None = None, driver: ServoDriver | None = None):
        self.ik = ik
        self.driver = driver

    def read(self) -> RobotObservation:
        if self.driver is None:
            raise NotImplementedError("Hardware observations require servo and IMU integration")
        return self.driver.read()

    def write_feet(self, targets: FootTargets) -> None:
        if self.ik is None or self.driver is None:
            raise NotImplementedError("Hardware IK, calibration and servo driver are pending")
        self.driver.write(normalize_joint_targets(self.ik.solve(targets)))

    def write_joints(self, targets: JointTargets) -> None:
        raise NotImplementedError("The hardware backend accepts foot targets only")
