"""将机体速度命令转换为 dog-v2 的前视动作参考。"""

import mujoco
import numpy as np

from dog_controller.config import LEGS
from dog_policy.bundle import GOAL_FRAMES, PERIOD_NS

from .backend import MujocoBackend
from .motion import Motion


DT = PERIOD_NS / 1e9
MAX_SPEED = 0.22
MAX_YAW = 0.65
FREQUENCY = 1.65
STANCE_RATIO = 0.65
STEP_HEIGHT = 0.035


class Gait:
    def __init__(self, backend: MujocoBackend, home: np.ndarray):
        model = backend.model
        self.home = home
        self.hips = np.array([model.body(f"{leg}_hip").pos for leg in LEGS])
        self.thighs = np.array([model.body(f"{leg}_thigh").pos for leg in LEGS])
        self.thigh_length = np.array([-model.body(f"{leg}_calf").pos[2] for leg in LEGS])
        foot_offsets = np.array([model.site(leg).pos for leg in LEGS])
        self.calf_length = -foot_offsets[:, 2]
        self.lateral = self.thighs[:, 1] + foot_offsets[:, 1]
        self.limits = backend.limits
        pose = mujoco.MjData(model)
        pose.qpos[backend.qpos] = home
        mujoco.mj_forward(model, pose)
        self.feet = np.array([pose.site(leg).xpos for leg in LEGS]) - pose.xpos[backend.root]
        self.feet[:, 2] = np.median(self.feet[:, 2])
        radius = model.geom("FL_foot_collision").size[0]
        self.height = float(-self.feet[0, 2] + radius)
        self.command = np.zeros(2)
        self.velocity = np.zeros(2)
        self.phase = 0.0
        self.previous = self._angles(self.feet).copy()

    def stand(self) -> Motion:
        angles = self._angles(self.feet)[None, :]
        zeros = np.zeros((1, 3))
        return Motion(
            angles, np.zeros_like(angles), np.array([[0., 0., self.height]]),
            np.array([[1., 0., 0., 0.]]), zeros, zeros, np.array([[0., 0., -1.]]),
            np.zeros((1, 34)),
        )

    def set_velocity(self, linear: float, yaw: float) -> None:
        if not np.isfinite([linear, yaw]).all():
            self.command.fill(0)
        else:
            self.command[:] = np.clip([linear, yaw], [-MAX_SPEED, -MAX_YAW], [MAX_SPEED, MAX_YAW])

    def reset(self) -> None:
        self.command.fill(0)
        self.velocity.fill(0)
        self.phase = 0.0
        self.previous = self._angles(self.feet)

    def goal(self) -> np.ndarray:
        # 前视各帧按当前限加速度推演。只推进首帧的规划状态。
        steps = np.arange(1, GOAL_FRAMES + 1)
        acceleration = np.array([0.55, 1.8])
        delta = self.command - self.velocity
        velocity = self.velocity + np.sign(delta) * np.minimum(
            np.abs(delta), steps[:, None] * DT * acceleration,
        )
        phases = self.phase + steps * DT * FREQUENCY
        legs = (phases[:, None] + np.array([0, .5, .5, 0])) % 1
        swing = np.clip((legs - STANCE_RATIO) / (1 - STANCE_RATIO), 0, 1)
        stride_position = np.where(
            legs < STANCE_RATIO, .5 - legs / STANCE_RATIO, -.5 + swing**2 * (3 - 2 * swing),
        )
        linear, yaw = velocity.T
        foot_x = linear[:, None] - yaw[:, None] * self.feet[:, 1]
        foot_y = yaw[:, None] * self.feet[:, 0]
        stride_scale = stride_position * STANCE_RATIO / FREQUENCY
        lift_scale = np.clip(np.maximum(np.abs(linear) / .09, np.abs(yaw) / .30), 0, 1)
        targets = np.broadcast_to(self.feet, (GOAL_FRAMES, *self.feet.shape)).copy()
        targets[..., 0] += stride_scale * foot_x
        targets[..., 1] += stride_scale * foot_y
        targets[..., 2] += STEP_HEIGHT * np.sin(np.pi * swing)**2 * lift_scale[:, None]
        angles = self._angles(targets)
        angle_velocity = np.diff(angles, axis=0, prepend=self.previous[None, :]) / DT

        features = np.zeros((GOAL_FRAMES, 34), np.float32)
        features[:, :12] = angles - self.home
        features[:, 12:24] = angle_velocity
        features[:, 26] = -1
        features[:, 29] = yaw
        features[:, 30] = linear
        features[:, 33] = self.height
        self.velocity[:] = velocity[0]
        self.phase = phases[0] % 1
        self.previous = angles[0].copy()
        return features.reshape(1, -1)

    def _angles(self, feet: np.ndarray) -> np.ndarray:
        # 求取朝下的髋关节分支及负角度的膝关节分支。
        offset = feet - self.hips
        x = offset[..., 0] - self.thighs[:, 0]
        y, z = offset[..., 1], offset[..., 2]
        down = -np.sqrt(np.maximum(y*y + z*z - self.lateral**2, 1e-12))
        hip = np.arctan2(z, y) - np.arctan2(down, self.lateral)
        z_plane = down - self.thighs[:, 2]
        l1, l2 = self.thigh_length, self.calf_length
        cosine = (x*x + z_plane*z_plane - l1*l1 - l2*l2) / (2*l1*l2)
        calf = -np.arccos(np.clip(cosine, -1, 1))
        thigh = np.arctan2(-x, -z_plane) - np.arctan2(l2*np.sin(calf), l1 + l2*np.cos(calf))
        angles = np.stack((hip, thigh, calf), axis=-1)
        angles = (angles + np.pi) % (2*np.pi) - np.pi
        return np.clip(angles.reshape((*angles.shape[:-2], 12)), self.limits[:, 0], self.limits[:, 1])
