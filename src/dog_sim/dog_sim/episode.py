"""以固定周期执行一个参考片段。"""

from dog_controller.controller import Controller
from dog_controller.interfaces import RobotObservation
from dog_policy.bundle import PERIOD_NS, Bundle
from dog_policy.tracker import Tracker

from .backend import MujocoBackend
from .config import MAX_GRAVITY_Z_ERROR, MAX_HEIGHT_ERROR, PHYSICS_STEP_NS, SUBSTEPS
from .motion import Motion


class Episode:
    def __init__(self, backend: MujocoBackend, bundle: Bundle, motion: Motion):
        if SUBSTEPS * PHYSICS_STEP_NS != PERIOD_NS:
            raise ValueError("Physics timestep and policy period do not match")
        self.backend = backend
        self.controller = Controller(backend)
        self.tracker = Tracker(bundle)
        self.motion = motion
        self.running = False
        self.frame = 0
        self.time_ns = 0

    def play(self) -> RobotObservation:
        self.running = False
        self.backend.reset(self.motion)
        self.tracker.reset()
        self.frame = 0
        observation = self.controller.read()
        self.running = True
        return observation

    def stop(self) -> None:
        self.running = False

    def step(self) -> RobotObservation:
        if not self.running:
            raise RuntimeError("Playback is paused")
        frame = self.frame
        observation = self._advance(
            self.motion.goal(frame), self.motion.root_position[frame, 2], self.motion.gravity_body[frame, 2],
        )
        if self.frame == len(self.motion):
            self.stop()
        return observation

    def _advance(self, goal, reference_height, reference_gravity_z) -> RobotObservation:
        self.controller.submit(self._joint_targets(goal))
        self.backend.advance(SUBSTEPS)
        self.frame += 1
        self.time_ns += PERIOD_NS
        observation = self.controller.read()
        height_error = abs(self.backend.root_position[2] - reference_height)
        gravity_error = abs(observation.projected_gravity[2] - reference_gravity_z)
        if height_error > MAX_HEIGHT_ERROR or gravity_error > MAX_GRAVITY_Z_ERROR:
            self.stop()
            raise RuntimeError(f"Root height or orientation error exceeded the limit at frame {self.frame}")
        return observation

    def _joint_targets(self, goal):
        return self.tracker.predict(self.controller.read(), goal)
