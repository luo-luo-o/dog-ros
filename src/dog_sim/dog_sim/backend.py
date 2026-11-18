"""安装后的 MJCF 与 URDF 绑定，以及同步物理后端。"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from dog_controller.config import BODY_FRAME, JOINT_NAMES
from dog_controller.interfaces import FootTargets, JointTargets, RobotObservation

from .config import PHYSICS_STEP_NS
from .motion import Motion


class MujocoBackend:
    def __init__(self, model_path: Path, urdf_path: Path):
        meshes = {f"meshes/{path.name}": path.read_bytes()
                  for path in (urdf_path.parent / "meshes").glob("*.stl")}
        self.model = mujoco.MjModel.from_xml_string(model_path.read_text(encoding="utf-8"), meshes)
        self.data = mujoco.MjData(self.model)
        self.root = self.model.body(BODY_FRAME).id
        roots = np.flatnonzero(self.model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)
        if len(roots) != 1 or self.model.jnt_bodyid[roots[0]] != self.root:
            raise ValueError("The MJCF must have a single free joint on the trunk")
        self.root_qpos = int(self.model.jnt_qposadr[roots[0]])
        self.root_qvel = int(self.model.jnt_dofadr[roots[0]])

        model = self.model
        joints = [model.joint(name).id for name in JOINT_NAMES]
        if model.njnt != len(joints) + 1 or model.nu != len(joints):
            raise ValueError("Unexpected joint or actuator count in the MJCF")
        if not np.all(model.jnt_type[joints] == mujoco.mjtJoint.mjJNT_HINGE):
            raise ValueError("Dog leg joints must be hinge joints")
        self.qpos = model.jnt_qposadr[joints]
        self.qvel = model.jnt_dofadr[joints]
        self.actuators = [model.actuator(name).id for name in JOINT_NAMES]
        if not np.all(model.actuator_trntype[self.actuators] == mujoco.mjtTrn.mjTRN_JOINT) or not np.array_equal(
            model.actuator_trnid[self.actuators, 0], joints
        ):
            raise ValueError("Incorrect MJCF actuator-to-joint mapping")
        self.limits = model.jnt_range[joints]
        self._check_urdf(urdf_path, joints)
        if not math.isclose(model.opt.timestep, PHYSICS_STEP_NS / 1e9, abs_tol=1e-12):
            raise ValueError("The MJCF timestep differs from the configured timestep")

    def _check_urdf(self, path, joints):
        revolute = [joint for joint in ET.parse(path).getroot().findall("joint")
                    if joint.attrib["type"] == "revolute"]
        by_name = {joint.attrib["name"]: joint for joint in revolute}
        if len(revolute) != len(JOINT_NAMES) or set(by_name) != set(JOINT_NAMES):
            raise ValueError("The URDF joint names differ from the controller joint names")
        for name, index, expected_range in zip(JOINT_NAMES, joints, self.limits):
            joint = by_name[name]
            axis = list(map(float, joint.find("axis").attrib["xyz"].split()))
            limits = joint.find("limit").attrib
            if not np.allclose(axis, self.model.jnt_axis[index]) or not np.allclose(
                [float(limits["lower"]), float(limits["upper"])], expected_range
            ) or not self.model.jnt_limited[index]:
                raise ValueError(f"The URDF and MJCF axes or limits differ for {name}")

    def reset(self, motion: Motion) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qpos] = np.clip(motion.position[0], self.limits[:, 0], self.limits[:, 1])
        self.data.qvel[self.qvel] = motion.velocity[0]
        self.data.qpos[self.root_qpos:self.root_qpos + 3] = motion.root_position[0]
        self.data.qpos[self.root_qpos + 3:self.root_qpos + 7] = motion.root_quaternion[0]
        self.data.qvel[self.root_qvel:self.root_qvel + 3] = motion.root_velocity_world[0]
        self.data.qvel[self.root_qvel + 3:self.root_qvel + 6] = motion.root_angular_body[0]
        self.data.ctrl[self.actuators] = self.data.qpos[self.qpos]
        mujoco.mj_forward(self.model, self.data)

    def advance(self, steps: int) -> None:
        start = self.data.time
        mujoco.mj_step(self.model, self.data, nstep=steps)
        if not math.isclose(self.data.time, start + steps * self.model.opt.timestep, abs_tol=1e-9):
            raise RuntimeError("Unexpected MuJoCo time after stepping")
        if not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel)):
            raise RuntimeError("MuJoCo state contains non-finite values")

    @property
    def root_position(self) -> np.ndarray:
        return self.data.qpos[self.root_qpos:self.root_qpos + 3]

    @property
    def root_quaternion(self) -> np.ndarray:
        return self.data.qpos[self.root_qpos + 3:self.root_qpos + 7]

    def read(self) -> RobotObservation:
        # 自由关节的后三个速度分量已经位于机体坐标系。
        w, x, y, z = self.root_quaternion
        gravity = (2 * (w * y - x * z), -2 * (y * z + w * x), 2 * (x * x + y * y) - 1)
        return RobotObservation(
            tuple(self.data.qpos[self.qpos]),
            tuple(self.data.qvel[self.qvel]),
            tuple(self.data.qvel[self.root_qvel + 3:self.root_qvel + 6]),
            gravity,
        )

    def write_joints(self, targets: JointTargets) -> None:
        self.data.ctrl[self.actuators] = targets.position

    def write_feet(self, targets: FootTargets) -> None:
        raise NotImplementedError("The MuJoCo backend accepts joint position targets only")
