"""加载并校验部署策略。"""

import hashlib
import json
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import onnxruntime as ort

from dog_controller.config import JOINT_NAMES


HISTORY_FRAMES = 10
GOAL_FRAMES = 10
PERIOD_NS = 20_000_000
INPUT_SHAPES = {"obs_history": [1, 420], "goal": [1, 340]}
OUTPUT_SHAPE = [1, 12]
MANIFEST = "manifest.json"
ACTIVE = "active"


def valid_name(name: str) -> str:
    if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", name):
        raise ValueError(f"Invalid policy version or motion clip name: {name!r}")
    return name


@dataclass(frozen=True)
class Bundle:
    name: str
    clip: str
    clip_sha256: str
    home: np.ndarray
    action_scale: float
    session: ort.InferenceSession

    @classmethod
    def load(cls, root: Path, version: str | None = None):
        if version is None:
            version = (root / ACTIVE).read_text(encoding="utf-8").strip()
        directory = root / valid_name(version)
        manifest = json.loads((directory / MANIFEST).read_text(encoding="utf-8"))
        for key, expected in {
            "name": version,
            "joint_names": list(JOINT_NAMES),
            "period_ns": PERIOD_NS,
            "history_frames": HISTORY_FRAMES,
            "goal_frames": GOAL_FRAMES,
            "input_shapes": INPUT_SHAPES,
            "output_shape": OUTPUT_SHAPE,
        }.items():
            if manifest[key] != expected:
                raise ValueError(f"Policy {version} has an incompatible {key}")

        home = np.asarray(manifest["home"], dtype=np.float32)
        scale = float(manifest["action_scale"])
        if home.shape != (len(JOINT_NAMES),) or not np.all(np.isfinite(home)):
            raise ValueError("Invalid policy home joint positions")
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError("Invalid policy action scale")

        clip = valid_name(manifest["default_clip"])
        clip_hash = manifest["clip_sha256"]
        if not isinstance(clip_hash, str) or not re.fullmatch(r"[a-f0-9]{64}", clip_hash):
            raise ValueError("Invalid default motion clip SHA-256")
        model = directory / manifest["model"]
        if model.parent != directory or model.resolve().parent != directory.resolve():
            raise ValueError("The ONNX file must be inside the version directory")
        payload = model.read_bytes()
        if hashlib.sha256(payload).hexdigest() != manifest["sha256"]:
            raise ValueError(f"ONNX checksum mismatch for policy {version}")

        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        session = ort.InferenceSession(payload, sess_options=options, providers=["CPUExecutionProvider"])
        actual_inputs = {tensor.name: (tensor.shape, tensor.type) for tensor in session.get_inputs()}
        expected_inputs = {name: (shape, "tensor(float)") for name, shape in INPUT_SHAPES.items()}
        outputs = [(tensor.name, tensor.shape, tensor.type) for tensor in session.get_outputs()]
        if actual_inputs != expected_inputs or outputs != [("action", OUTPUT_SHAPE, "tensor(float)")]:
            raise ValueError("ONNX inputs or outputs do not match the policy interface")

        bundle = cls(version, clip, clip_hash, home, scale, session)
        bundle.infer(*(np.zeros(shape, dtype=np.float32) for shape in INPUT_SHAPES.values()))
        return bundle

    def infer(self, obs_history: np.ndarray, goal: np.ndarray) -> np.ndarray:
        inputs = {"obs_history": obs_history, "goal": goal}
        for name, values in inputs.items():
            if list(values.shape) != INPUT_SHAPES[name] or not np.all(np.isfinite(values)):
                raise ValueError(f"Invalid policy input {name}")
        action = self.session.run(["action"], inputs)[0][0]
        if not np.all(np.isfinite(action)):
            raise ValueError("Policy output contains non-finite values")
        return action
