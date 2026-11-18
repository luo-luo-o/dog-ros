"""载入狗动作片段并构造训练所用的参考窗口。"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from dog_controller.config import BODY_FRAME, JOINT_NAMES
from dog_policy.bundle import GOAL_FRAMES, PERIOD_NS


def rotate_inverse(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    imaginary = -quaternion[..., 1:]
    twice_cross = 2 * np.cross(imaginary, vector)
    return vector + quaternion[..., :1] * twice_cross + np.cross(imaginary, twice_cross)


@dataclass(frozen=True)
class Motion:
    position: np.ndarray
    velocity: np.ndarray
    root_position: np.ndarray
    root_quaternion: np.ndarray
    root_velocity_world: np.ndarray
    root_angular_body: np.ndarray
    gravity_body: np.ndarray
    features: np.ndarray

    @classmethod
    def load(cls, path: Path, home: np.ndarray):
        with np.load(path, allow_pickle=False) as archive:
            names = tuple(archive["joint_names"].tolist())
            if len(names) != len(JOINT_NAMES) or set(names) != set(JOINT_NAMES):
                raise ValueError("The motion clip must contain exactly 12 unique known joints")
            bodies = archive["body_names"].tolist()
            if bodies.count(BODY_FRAME) != 1:
                raise ValueError("The motion clip must contain exactly one trunk")
            root = bodies.index(BODY_FRAME)
            fps = archive["fps"]
            if fps.shape != () or float(fps) != 1e9 / PERIOD_NS:
                raise ValueError("The motion clip must run at 50 Hz")

            position = np.asarray(archive["joint_pos"], dtype=np.float32)
            if position.ndim != 2 or position.shape[0] == 0:
                raise ValueError("The motion clip has no playable frames")
            frames = position.shape[0]

            def read(key, shape):
                data = np.asarray(archive[key], dtype=np.float32)
                if data.shape != shape or not np.all(np.isfinite(data)):
                    raise ValueError(f"Motion clip field {key} has an invalid shape or value")
                return data

            indices = [names.index(name) for name in JOINT_NAMES]
            position = read("joint_pos", (frames, len(names)))[:, indices]
            velocity = read("joint_vel", (frames, len(names)))[:, indices]
            root_position = read("body_pos_w", (frames, len(bodies), 3))[:, root]
            quaternion = read("body_quat_w", (frames, len(bodies), 4))[:, root]
            root_velocity = read("body_lin_vel_w", (frames, len(bodies), 3))[:, root]
            root_angular = read("body_ang_vel_w", (frames, len(bodies), 3))[:, root]

        if not np.allclose(np.linalg.norm(quaternion, axis=1), 1, atol=1e-3):
            raise ValueError("Motion clip body quaternions must have unit length")
        gravity = rotate_inverse(quaternion, np.array([0, 0, -1], dtype=np.float32))
        angular_body = rotate_inverse(quaternion, root_angular)
        w, x, y, z = quaternion.T
        half_yaw = 0.5 * np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        zero = np.zeros_like(half_yaw)
        heading = np.column_stack((np.cos(half_yaw), zero, zero, np.sin(half_yaw)))
        features = np.column_stack((position - home, velocity, gravity, angular_body,
                                    rotate_inverse(heading, root_velocity), root_position[:, 2]))
        return cls(position, velocity, root_position, quaternion, root_velocity,
                   angular_body, gravity, features)

    def __len__(self) -> int:
        return self.features.shape[0]

    def goal(self, frame: int) -> np.ndarray:
        indices = np.minimum(frame + np.arange(GOAL_FRAMES), len(self) - 1)
        return self.features[indices].reshape(1, -1)
