"""在当前工作空间中评测策略与 MuJoCo 后端。结果以 UTF-8 JSON 写入标准输出。"""

import argparse
import csv
import hashlib
import json
from pathlib import Path
from time import monotonic

import mujoco
import numpy as np
import onnxruntime

from dog_policy.bundle import Bundle
from dog_sim.backend import MujocoBackend
from dog_sim.episode import Episode
from dog_sim.motion import Motion


def sha256_file(path):
    with path.open("rb") as file:
        digest = hashlib.sha256()
        for chunk in iter(lambda: file.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def yaw(quaternion):
    w, x, y, z = quaternion
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def benchmark_clip(backend, policy, path, entry):
    xy_errors, yaw_errors, joint_squared_errors, height_errors, gravity_z_errors = [], [], [], [], []
    failure_reason = ""
    episode = None
    try:
        motion = Motion.load(path, policy.home)
        if len(motion) != int(entry["frames"]):
            raise ValueError("Motion frame count does not match the manifest")
        episode = Episode(backend, policy, motion)
        episode.play()
    except (AttributeError, KeyError, OSError, RuntimeError, ValueError) as error:
        failure_reason = str(error)
    while episode is not None and episode.running:
        frame = episode.frame
        try:
            observation = episode.step()
        except (RuntimeError, ValueError) as error:
            failure_reason = str(error)
            episode.stop()
            if episode.frame == frame:
                break
            try:
                observation = episode.controller.read()
            except ValueError:
                break
        xy_errors.append(np.linalg.norm(backend.root_position[:2] - motion.root_position[frame, :2]))
        yaw_delta = yaw(backend.root_quaternion) - yaw(motion.root_quaternion[frame])
        yaw_errors.append(abs((yaw_delta + np.pi) % (2 * np.pi) - np.pi))
        joint_squared_errors.append(np.square(np.asarray(observation.position) - motion.position[frame]).mean())
        height_errors.append(abs(backend.root_position[2] - motion.root_position[frame, 2]))
        gravity_z_errors.append(abs(observation.projected_gravity[2] - motion.gravity_body[frame, 2]))
    failed = bool(failure_reason)
    steps = episode.frame if episode is not None else 0
    return {
        "clip": path.stem,
        "group": entry["group"],
        "category": entry["category"],
        "frames": int(entry["frames"]),
        "steps": steps,
        "failed": failed,
        "fail_step": steps if failed else -1,
        "failure_reason": failure_reason,
        "mean_xy_drift_m": float(np.mean(xy_errors)) if xy_errors else None,
        "mean_yaw_drift_rad": float(np.mean(yaw_errors)) if yaw_errors else None,
        "joint_rmse_rad": float(np.sqrt(np.mean(joint_squared_errors))) if joint_squared_errors else None,
        "max_height_error_m": float(max(height_errors)) if height_errors else None,
        "max_gravity_z_error": float(max(gravity_z_errors)) if gravity_z_errors else None,
    }


def summarize_group(results, group):
    clips = [row for row in results if group == "all" or row["group"] == group]
    return {
        "group": group,
        "clip_count": len(clips),
        "completed_clip_count": sum(not row["failed"] and row["steps"] == row["frames"] for row in clips),
        "failed_clip_count": sum(row["failed"] for row in clips),
        "frames": sum(row["frames"] for row in clips),
        "steps": sum(row["steps"] for row in clips),
    }


def main():
    parser = argparse.ArgumentParser(description="Benchmark the current workspace with ONNX Runtime and MuJoCo.")
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--motion-dir", type=Path, help="motion library with manifest.csv; defaults to the bundled deployment clips")
    parser.add_argument("--clip", help="Evaluate only the named clip, without its extension")
    parser.add_argument("--policy-version", help="Policy version to evaluate (defaults to the active version)")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    model = workspace / "src/dog_sim/model/dog.xml"
    urdf_path = workspace / "src/dog_controller/model/dog/dog.urdf"
    motion_dir = args.motion_dir.resolve() if args.motion_dir else workspace / "motions/dog"
    manifest_path = motion_dir / "manifest.csv"
    policy_dir = workspace / "policies/deploy"
    policy = Bundle.load(policy_dir, args.policy_version)
    policy_manifest_path = policy_dir / policy.name / "manifest.json"
    policy_manifest = json.loads(policy_manifest_path.read_text(encoding="utf-8"))
    backend = MujocoBackend(model, urdf_path)
    with manifest_path.open(encoding="utf-8", newline="") as file:
        entries = [
            row for row in csv.DictReader(file)
            if row["keep"] == "1" and (args.clip is None or args.clip == row["name"])
        ]
    if not entries:
        raise ValueError("No matching clip found in the manifest")
    start_time = monotonic()
    results = [
        benchmark_clip(backend, policy, motion_dir / (entry["name"] + ".npz"), entry)
        for entry in entries
    ]
    print(json.dumps({
        "policy_version": policy.name,
        "onnx_sha256": policy_manifest["sha256"],
        "motion_manifest_sha256": sha256_file(manifest_path),
        "mjcf_sha256": sha256_file(model),
        "urdf_sha256": sha256_file(urdf_path),
        "mujoco_version": mujoco.__version__,
        "onnxruntime_version": onnxruntime.__version__,
        "numpy_version": np.__version__,
        "evaluation_duration_s": monotonic() - start_time,
        "clips": results,
        "group_summary": [summarize_group(results, group) for group in ("all", "holdout", "train", "hard")],
    }, ensure_ascii=False, allow_nan=False, indent=2))


if __name__ == "__main__":
    main()
