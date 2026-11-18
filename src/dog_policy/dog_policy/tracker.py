"""构造与训练一致的分项观测历史。"""

from collections import deque

import numpy as np

from dog_controller.config import JOINT_NAMES
from dog_controller.interfaces import JointTargets, RobotObservation

from .bundle import HISTORY_FRAMES, Bundle


class Tracker:
    def __init__(self, bundle: Bundle):
        self.bundle = bundle
        self.history = deque(maxlen=HISTORY_FRAMES)
        self.previous_action = np.zeros(len(JOINT_NAMES), dtype=np.float32)

    def reset(self) -> None:
        self.history.clear()
        self.previous_action.fill(0)

    def predict(self, observation: RobotObservation, goal: np.ndarray) -> JointTargets:
        terms = (
            np.asarray(observation.position, dtype=np.float32) - self.bundle.home,
            np.asarray(observation.velocity, dtype=np.float32),
            np.asarray(observation.angular_velocity, dtype=np.float32),
            np.asarray(observation.projected_gravity, dtype=np.float32),
            self.previous_action,
        )
        if not self.history:
            self.history.extend([terms] * HISTORY_FRAMES)
        else:
            self.history.append(terms)
        history = np.concatenate([np.asarray(term).reshape(-1) for term in zip(*self.history)])
        action = self.bundle.infer(history[None], goal)
        self.previous_action = action
        position = self.bundle.home + self.bundle.action_scale * action
        return JointTargets(JOINT_NAMES, tuple(map(float, position)))
