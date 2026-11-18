"""从指定的本地训练环境导出初始策略，并用真实动作输入验证。"""

import argparse
import csv
import hashlib
import json
import shutil
import subprocess
import sys
from io import BytesIO
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import torch


def main():
    parser = argparse.ArgumentParser(description="Export the initial checkpoint and validate it against recorded motion.")
    parser.add_argument("training", metavar="TRAINING_REPO")
    parser.add_argument("--checkpoint", type=Path, help="defaults to logs/dog-v2/ckpt_latest.pt in the training repository")
    parser.add_argument("--clip", default="ai4a_D1_ex03_KAN02_012_s20", help="clip name in the training manifest")
    parser.add_argument("--output", type=Path, help="defaults to out/dog-ros-export in the training repository")
    args = parser.parse_args()
    training = Path(args.training).expanduser().resolve()
    sys.path.insert(0, str(training / "src"))
    from supermimic.envs.mdp.motion_library import MotionLibrary
    from supermimic.eval.benchmark import load_policy_from_checkpoint
    from supermimic.kinematics.rotations import projected_gravity, quat_apply_inverse
    from supermimic.robots.dog import HOME, JOINT_NAMES

    checkpoint = args.checkpoint or training / "logs/dog-v2/ckpt_latest.pt"
    clip_name = args.clip
    training_motions = training / "data/npz/dog"
    with (training_motions / "manifest.csv").open(encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        fields = reader.fieldnames
        entries = [row for row in reader if row["name"] == clip_name and row["keep"] == "1"]
    if not fields or len(entries) != 1:
        parser.error("the requested clip must occur once among the retained training clips")
    clip = training_motions / f"{clip_name}.npz"
    export_root = args.output or training / "out/dog-ros-export"
    output = export_root / "policies/deploy/dog-v2"
    torch.set_num_threads(1)
    policy, config = load_policy_from_checkpoint(checkpoint, "cpu")
    assert config["robot"] == "dog" and not config["env"]["anchor_obs"]
    assert config["env"]["stiffness"] is None and config["env"]["action_scale"] is None
    home = torch.tensor([HOME[name] for name in JOINT_NAMES])
    library = MotionLibrary([clip], ("trunk",), home, device="cpu", root_body="trunk")
    count = library.num_frames
    frames = torch.arange(count)
    reference = library.ref(frames)
    terms = (
        reference.joint_pos - home,
        reference.joint_vel,
        quat_apply_inverse(reference.root_quat_w, reference.root_ang_vel_w),
        projected_gravity(reference.root_quat_w),
    )
    history = policy.cfg.history
    indices = (frames[:, None] - torch.arange(history - 1, -1, -1)).clamp(min=0)
    goal = library.goal_window(frames, config["env"]["future_frames"])
    obs = torch.cat([term[indices[0]].reshape(1, -1) for term in terms] + [torch.zeros(1, 120)], dim=1)
    model_bytes = BytesIO()
    torch.onnx.export(
        policy, (obs, goal[:1]), model_bytes, dynamo=False, opset_version=17,
        input_names=["obs_history", "goal"], output_names=["action"],
    )
    model = model_bytes.getvalue()
    onnx.checker.check_model(onnx.load_model_from_string(model))
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    session = ort.InferenceSession(model, sess_options=options, providers=["CPUExecutionProvider"])
    actions = torch.zeros(count + 1, 12)
    max_error = 0.0
    observations = []
    expected_actions = []
    with torch.inference_mode():
        for frame, window in enumerate(goal):
            idx = indices[frame]
            obs = torch.cat([term[idx].reshape(1, -1) for term in (*terms, actions)], dim=1)
            expected = policy(obs, window[None])
            actual = session.run(None, {"obs_history": obs.numpy(), "goal": window[None].numpy()})[0]
            np.testing.assert_allclose(actual, expected.numpy(), atol=1e-5, rtol=1e-4)
            max_error = max(max_error, float(np.max(np.abs(actual - expected.numpy()))))
            actions[frame + 1] = expected[0]
            observations.append(obs.numpy()[0].copy())
            expected_actions.append(expected.numpy()[0].copy())
    with checkpoint.open("rb") as checkpoint_file:
        checkpoint_sha256 = hashlib.file_digest(checkpoint_file, "sha256").hexdigest()
    manifest = {
        "name": "dog-v2",
        "model": "policy.onnx",
        "sha256": hashlib.sha256(model).hexdigest(),
        "checkpoint_sha256": checkpoint_sha256,
        "source_revision": subprocess.check_output(["git", "-C", str(training), "rev-parse", "HEAD"], text=True).strip(),
        "default_clip": clip_name,
        "clip_sha256": hashlib.sha256(clip.read_bytes()).hexdigest(),
        "joint_names": list(JOINT_NAMES),
        "home": home.tolist(),
        "period_ns": 20_000_000,
        "history_frames": history,
        "goal_frames": config["env"]["future_frames"],
        "action_scale": 0.25,
        "input_shapes": {"obs_history": [1, 420], "goal": [1, 340]},
        "output_shape": [1, 12],
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / manifest["model"]).write_bytes(model)
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    motions = export_root / "motions/dog"
    motions.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(clip, motions / clip.name)
    with (motions / "manifest.csv").open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(entries)

    fixtures = export_root / "tests/fixtures"
    fixtures.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(fixtures / "policy_reference.npz", obs_history=np.array(observations), goal=goal.numpy(),
                        action=np.array(expected_actions), position=reference.joint_pos.numpy(),
                        velocity=reference.joint_vel.numpy(), angular_velocity=terms[2].numpy(),
                        gravity=terms[3].numpy())
    print(f"Validated {count} frames; maximum absolute error {max_error:.8g}")
    print(f"Deployment export: {export_root.resolve()}")
    print(f"Policy SHA-256 {manifest['sha256']}")


if __name__ == "__main__":
    main()
