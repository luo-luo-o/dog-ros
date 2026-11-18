"""持续步进的速度控制会话。"""

from math import atan2, pi, remainder

import numpy as np

from dog_controller.interfaces import JointTargets, RobotObservation
from dog_policy.bundle import Bundle

from .backend import MujocoBackend
from .config import TELEOP_HEADING_GAIN, TELEOP_YAW_DEADBAND
from .episode import Episode
from .gait import Gait


class DrivingEpisode(Episode):
    def __init__(self, backend: MujocoBackend, bundle: Bundle):
        self.gait = Gait(backend, bundle.home)
        self.requested_yaw = 0.0
        self.heading_target = None
        super().__init__(backend, bundle, self.gait.stand())

    def play(self) -> RobotObservation:
        self.gait.reset()
        self.requested_yaw = 0.0
        self.heading_target = None
        return super().play()

    def set_velocity(self, linear: float, yaw: float) -> None:
        if abs(yaw) <= TELEOP_YAW_DEADBAND:
            yaw = 0.0
        self.gait.set_velocity(linear, yaw)
        self.requested_yaw = float(self.gait.command[1])
        if self.requested_yaw != 0 or self.gait.command[0] == 0:
            self.heading_target = None

    def step(self) -> RobotObservation:
        if not self.running:
            raise RuntimeError("Driving is paused")
        if self.requested_yaw == 0 and self.gait.command[0] != 0:
            # 每次开始直行或结束转向时锁定实际朝向。后续速度更新沿用该目标。
            w, x, y, z = self.backend.root_quaternion
            heading = atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
            if self.heading_target is None:
                self.heading_target = heading
            error = remainder(self.heading_target - heading, 2 * pi)
            self.gait.set_velocity(self.gait.command[0], TELEOP_HEADING_GAIN * error)
        return self._advance(self.gait.goal(), self.gait.height, -1)

    def _joint_targets(self, goal) -> JointTargets:
        prediction = super()._joint_targets(goal)
        reference = goal[0, :12] + self.gait.home
        # 合成步态与训练片段的分布不同，遥控模式约束策略对参考的修正量。
        correction = np.clip(0.2 * (np.asarray(prediction.position) - reference), -.09, .09)
        targets = np.clip(reference + correction, self.backend.limits[:, 0], self.backend.limits[:, 1])
        return JointTargets(prediction.names, tuple(map(float, targets)))
