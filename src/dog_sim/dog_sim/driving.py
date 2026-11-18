"""持续步进的速度控制会话。"""

import numpy as np

from dog_controller.interfaces import JointTargets, RobotObservation
from dog_policy.bundle import Bundle

from .backend import MujocoBackend
from .episode import Episode
from .gait import Gait


class DrivingEpisode(Episode):
    def __init__(self, backend: MujocoBackend, bundle: Bundle):
        self.gait = Gait(backend, bundle.home)
        super().__init__(backend, bundle, self.gait.stand())

    def play(self) -> RobotObservation:
        self.gait.reset()
        return super().play()

    def set_velocity(self, linear: float, yaw: float) -> None:
        self.gait.set_velocity(linear, yaw)

    def step(self) -> RobotObservation:
        if not self.running:
            raise RuntimeError("Driving is paused")
        return self._advance(self.gait.goal(), self.gait.height, -1)

    def _joint_targets(self, goal) -> JointTargets:
        prediction = super()._joint_targets(goal)
        reference = goal[0, :12] + self.gait.home
        # 合成步态与训练片段的分布不同，遥控模式约束策略对参考的修正量。
        correction = np.clip(0.2 * (np.asarray(prediction.position) - reference), -.09, .09)
        targets = np.clip(reference + correction, self.backend.limits[:, 0], self.backend.limits[:, 1])
        return JointTargets(prediction.names, tuple(map(float, targets)))
