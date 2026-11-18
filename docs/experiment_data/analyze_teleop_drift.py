"""对左右几何、参考站姿和策略修正做独立消融。"""

import hashlib
import json
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from dog_controller.config import JOINT_NAMES
from dog_controller.interfaces import JointTargets
from dog_policy.bundle import Bundle
from dog_sim.backend import MujocoBackend
from dog_sim.driving import DrivingEpisode


ROOT = Path(__file__).resolve().parents[2]
MJCF = ROOT / "src/dog_sim/model/dog.xml"
URDF = ROOT / "src/dog_controller/model/dog/dog.urdf"
JOINT_REFLECTION = np.array([-1, 1, 1] * 4)
JOINT_SWAP = np.array([3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def yaw(quaternion):
    w, x, y, z = quaternion
    return np.arctan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def make_symmetric_model(path):
    # 右侧碰撞形状与惯量使用左侧镜像。控制器读取的关节轴及限位保持一致。
    tree = ET.parse(MJCF)
    for left, right in (("FL", "FR"), ("RL", "RR")):
        for segment in ("hip", "thigh", "calf"):
            source = tree.find(f".//body[@name='{left}_{segment}']")
            target = tree.find(f".//body[@name='{right}_{segment}']")
            source_items = {item.attrib["name"][2:]: item for item in source.findall("geom")
                            if item.attrib["name"].endswith("_collision")}
            pairs = [(source, target), (source.find("inertial"), target.find("inertial"))]
            pairs += [(source_items[item.attrib["name"][2:]], item) for item in target.findall("geom")
                      if item.attrib["name"].endswith("_collision")]
            for origin, destination in pairs:
                for field, reflection in (
                    ("pos", (1, -1, 1)), ("fromto", (1, -1, 1, 1, -1, 1)),
                    ("fullinertia", (1, 1, 1, -1, 1, -1)),
                ):
                    if field in origin.attrib:
                        destination.set(field, " ".join(
                            str(float(value) * sign)
                            for value, sign in zip(origin.attrib[field].split(), reflection, strict=True)
                        ))
    tree.write(path, encoding="utf-8")


class Trial(DrivingEpisode):
    def __init__(self, backend, bundle, symmetric_stance, correction):
        super().__init__(backend, bundle)
        self.use_correction = correction
        if symmetric_stance:
            feet = self.gait.feet.copy()
            self.gait.feet[:] = (feet + feet[[1, 0, 3, 2]] * [1, -1, 1]) / 2
            self.motion = self.gait.stand()

    def set_velocity(self, linear, yaw):
        # 历史对照直接向步态规划器下发速度，由实验本身决定是否施加反馈。
        self.gait.set_velocity(linear, yaw)

    def step(self):
        return self._advance(self.gait.goal(), self.gait.height, -1)

    def _joint_targets(self, goal):
        if self.use_correction:
            return super()._joint_targets(goal)
        position = goal[0, :12] + self.gait.home
        return JointTargets(JOINT_NAMES, tuple(map(float, position)))


def run_trial(model, bundle, speed, phase, symmetric_stance, correction, feedback):
    backend = MujocoBackend(model, URDF)
    trial = Trial(backend, bundle, symmetric_stance, correction)
    trial.play()
    for _ in range(50):
        trial.step()
    initial = backend.root_position[:2].copy()
    heading = yaw(backend.root_quaternion.copy())
    c, s = np.cos(heading), np.sin(heading)
    basis = np.array([[c, s], [-s, c]])
    if phase is not None:
        trial.gait.phase = phase
    trail = []
    failure = ""
    body_yaw = 0.0
    for frame in range(500):
        xy = basis @ (backend.root_position[:2] - initial)
        theta = yaw(backend.root_quaternion) - heading
        body_yaw = float(np.arctan2(np.sin(theta), np.cos(theta)))
        yaw_command = 0.0
        if feedback == "yaw":
            yaw_command = -1.8 * body_yaw
        elif feedback == "line":
            yaw_command = 1.8 * (np.arctan2(-xy[1], .45) - body_yaw)
        trial.set_velocity(speed, yaw_command)
        try:
            trial.step()
        except RuntimeError as error:
            failure = str(error)
            break
        xy = basis @ (backend.root_position[:2] - initial)
        step_yaw = yaw(backend.root_quaternion) - heading
        angle = float(np.arctan2(np.sin(step_yaw), np.cos(step_yaw)))
        trail.append((float(xy[0]), float(xy[1]), float(np.degrees(angle))))
    samples = {str(second): trail[second * 50 - 1] for second in (2, 5, 10) if len(trail) >= second * 50}
    return {
        "model": "original" if model == MJCF else "mirrored_physics",
        "symmetric_stance": symmetric_stance,
        "policy_correction": correction,
        "command_m_s": speed,
        "start_phase": phase,
        "feedback": feedback,
        "completed_frames": len(trail),
        "failure": failure,
        "samples_forward_m_left_m_left_yaw_deg": samples,
        "end": trail[-1] if trail else None,
    }


def reflect_joints(values):
    return values[..., JOINT_SWAP] * JOINT_REFLECTION


def policy_reflection_error(bundle):
    # 镜像绝对关节角，再用原策略的 HOME 转回相对输入。
    backend = MujocoBackend(MJCF, URDF)
    trial = Trial(backend, bundle, True, True)
    trial.play()
    trial.set_velocity(.22, 0.)
    errors = []
    for index in range(200):
        goal = trial.gait.goal()
        trial._advance(goal, trial.gait.height, -1)
        if index < 50 or index % 5:
            continue
        q, dq, angular, gravity, actions = [np.array(term) for term in zip(*trial.tracker.history)]
        home, scale = bundle.home, bundle.action_scale
        mirrored_observation = (
            reflect_joints(q + home) - home,
            reflect_joints(dq),
            angular * [-1, 1, -1],
            gravity * [1, -1, 1],
            (reflect_joints(home + scale * actions) - home) / scale,
        )
        history = np.concatenate([term.ravel() for term in mirrored_observation])[None].astype(np.float32)
        reference = goal.reshape(-1, 34).copy()
        reference[:, :12] = reflect_joints(reference[:, :12] + home) - home
        reference[:, 12:24] = reflect_joints(reference[:, 12:24])
        reference[:, 24:27] *= [1, -1, 1]
        reference[:, 27:30] *= [-1, 1, -1]
        reference[:, 30:33] *= [1, -1, 1]
        mirrored_target = home + scale * bundle.infer(history, reference.reshape(1, -1))
        target = home + scale * trial.tracker.previous_action
        errors.append(np.abs(target - reflect_joints(mirrored_target)))
    return {
        "samples": len(errors),
        "mean_abs_target_error_rad": float(np.mean(errors)),
        "per_joint_mean_abs_target_error_rad": dict(zip(JOINT_NAMES, np.mean(errors, axis=0).tolist())),
        "max_abs_target_error_rad": float(np.max(errors)),
    }


def main():
    bundle = Bundle.load(ROOT / "policies/deploy", "dog-v2")
    backend = MujocoBackend(MJCF, URDF)
    gait = Trial(backend, bundle, False, True).gait
    left_right_difference = (gait.feet - gait.feet[[1, 0, 3, 2]] * [1, -1, 1]) * 1000
    mirrored_home = bundle.home[JOINT_SWAP] * JOINT_REFLECTION
    result = {
        "mujoco_version": mujoco.__version__,
        "onnx_sha256": sha256(ROOT / "policies/deploy/dog-v2/policy.onnx"),
        "mjcf_sha256": sha256(MJCF),
        "urdf_sha256": sha256(URDF),
        "home_left_minus_reflected_right_deg": dict(zip(JOINT_NAMES, np.degrees(bundle.home - mirrored_home).tolist())),
        "nominal_feet_m": gait.feet.tolist(),
        "nominal_feet_mirror_residual_mm": left_right_difference.tolist(),
        "policy_reflection_error": policy_reflection_error(bundle),
        "trials": [],
    }
    with tempfile.TemporaryDirectory() as directory:
        symmetric = Path(directory) / "mirrored.xml"
        make_symmetric_model(symmetric)
        for model in (MJCF, symmetric):
            for stance in (False, True):
                for correction in (False, True):
                    for speed in (.14, .22):
                        for phase in (None, 0., .25, .50, .75):
                            result["trials"].append(run_trial(model, bundle, speed, phase, stance, correction, "none"))
        for feedback in ("yaw", "line"):
            for speed in (.14, .22):
                for phase in (None, 0., .25, .50, .75):
                    result["trials"].append(run_trial(MJCF, bundle, speed, phase, False, True, feedback))
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
