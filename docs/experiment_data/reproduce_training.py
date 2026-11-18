"""用原训练环境复现控制策略的两组基准。结果以 UTF-8 JSON 写入标准输出。"""

import argparse
from contextlib import redirect_stdout
import csv
import json
from pathlib import Path
import sys
from time import monotonic


CANDIDATE_CLIPS = (
    "ai4a_D1_ex03_KAN02_012_s20",
    "ai4a_D1_ex03_KAN02_012_s20_mirror",
    "tencent_dog_hit_001_s2",
    "ai4a_D1_013_KAN01_001_s0_mirror",
    "ai4a_D1_004_KAN01_001_s0",
)

REWARD_NAMES = {
    "motion_global_root_pos": "root_position_reward",
    "motion_global_root_ori": "root_orientation_reward",
    "motion_body_pos": "body_position_reward",
    "motion_body_ori": "body_orientation_reward",
    "motion_body_lin_vel": "body_linear_velocity_reward",
    "motion_body_ang_vel": "body_angular_velocity_reward",
    "action_rate_l2": "action_rate_penalty",
    "joint_limit": "joint_limit_penalty",
}


def benchmark_training(policy, config, motion_dir):
    from supermimic.eval.benchmark import run_benchmark, summarize

    with (motion_dir / "manifest.csv").open(encoding="utf-8", newline="") as data:
        clip_entries = {row["name"]: row for row in csv.DictReader(data) if row["keep"] == "1"}
    groups = ("holdout", "train,hard")
    batches = []
    for group in groups:
        files = [
            str(motion_dir / (name + ".npz"))
            for name, entry in clip_entries.items()
            if entry["group"] in group.split(",")
        ]
        start_time = monotonic()
        env = config["env"]
        results = run_benchmark(
            policy,
            files,
            device="cpu",
            robot=config["robot"],
            num_envs=min(48, len(files)),
            future_frames=env["future_frames"],
            anchor_obs=env["anchor_obs"],
            support_foot_threshold=env.get("support_foot_threshold"),
            root_height_threshold=env.get("root_height_threshold"),
            root_tilt_threshold=env.get("root_tilt_threshold"),
        )
        summary = summarize(results)
        batches.append({
            "group": group,
            "duration_s": monotonic() - start_time,
            "clip_count": summary["clips"],
            "failed_clip_count": summary["failed"],
            "mean_normalized_return": summary["mean_norm_return"],
            "frame_weighted_normalized_return": summary["step_weighted_norm_return"],
            "clips": [
                {
                    "clip": clip.name,
                    "group": clip_entries[clip.name]["group"],
                    "category": clip_entries[clip.name]["category"],
                    "frames": clip.frames,
                    "steps": clip.steps,
                    "failed": clip.failed,
                    "fail_step": clip.fail_step,
                    "normalized_return": clip.norm_return,
                    "survived_normalized_return": clip.norm_return_survived,
                    "mean_xy_drift_m": clip.drift_xy,
                    "mean_yaw_drift_rad": clip.drift_yaw,
                    **{REWARD_NAMES[key]: value for key, value in clip.terms.items()},
                }
                for clip in sorted(results, key=lambda item: (item.failed, -item.norm_return, item.drift_xy, item.name))
            ],
        })
    return batches


def benchmark_native(policy, motion_dir):
    import mujoco
    import numpy as np
    import torch
    from mjlab.scene import Scene
    from supermimic.envs.mdp.motion_library import RefState, goal_features
    from supermimic.envs.tracking_env_cfg import SuperMimicEnvCfgArgs, make_env_cfg
    from supermimic.kinematics.rotations import quat_apply_inverse
    from supermimic.robots.dog import HOME, JOINT_NAMES

    paths = [motion_dir / (clip + ".npz") for clip in CANDIDATE_CLIPS]
    config = make_env_cfg(SuperMimicEnvCfgArgs(
        npz_files=[str(path) for path in paths],
        robot="dog",
        num_envs=1,
        domain_randomization=False,
        reset_noise=False,
        obs_noise=False,
        action_noise_std=0,
        record_transitions=False,
    ))
    scene = Scene(config.scene, device="cpu")
    model = scene.spec.compile()
    config.sim.mujoco.apply(model)

    root_joint = model.joint(0)
    root_qpos = np.arange(root_joint.qposadr[0], root_joint.qposadr[0] + 7)
    root_qvel = np.arange(root_joint.dofadr[0], root_joint.dofadr[0] + 6)
    joints = [model.joint("robot/" + name) for name in JOINT_NAMES]
    joint_qpos = np.array([entry.qposadr[0] for entry in joints])
    joint_qvel = np.array([entry.dofadr[0] for entry in joints])
    actuator_by_joint = {int(joint_id): index for index, (joint_id, _) in enumerate(model.actuator_trnid)}
    actuators = [actuator_by_joint[entry.id] for entry in joints]
    home = np.array([HOME[name] for name in JOINT_NAMES], dtype=np.float32)
    gravity = np.array([0.0, 0.0, -1.0])
    conjugate_sign = np.array([1.0, -1.0, -1.0, -1.0])

    results = []
    for path in paths:
        with np.load(path, allow_pickle=False) as data:
            trunk = list(data["body_names"]).index("trunk")
            reference_joints = [list(data["joint_names"]).index(name) for name in JOINT_NAMES]
            reference_root_position = data["body_pos_w"][:, trunk]
            reference_root_quaternion = data["body_quat_w"][:, trunk]
            reference_joint_position = data["joint_pos"][:, reference_joints]
            reference_root_linear_velocity = data["body_lin_vel_w"][:, trunk]
            reference_root_angular_velocity = data["body_ang_vel_w"][:, trunk]
            reference_joint_velocity = data["joint_vel"][:, reference_joints]
        reference_components = (
            reference_root_position,
            reference_root_quaternion,
            reference_joint_position,
            reference_root_linear_velocity,
            reference_root_angular_velocity,
            reference_joint_velocity,
        )
        reference = RefState(*(
            torch.as_tensor(component, dtype=torch.float32) for component in reference_components
        ))
        goal_features_by_frame = goal_features(reference, torch.as_tensor(home)).numpy()
        simulation = mujoco.MjData(model)
        simulation.qpos[root_qpos] = np.r_[reference_root_position[0], reference_root_quaternion[0]]
        simulation.qpos[joint_qpos] = reference_joint_position[0]
        body_angular_velocity = quat_apply_inverse(reference.root_quat_w[0], reference.root_ang_vel_w[0]).numpy()
        simulation.qvel[root_qvel] = np.r_[reference_root_linear_velocity[0], body_angular_velocity]
        simulation.qvel[joint_qvel] = reference_joint_velocity[0]
        mujoco.mj_forward(model, simulation)

        projected_gravity = np.empty(3)
        history = []
        action = np.zeros(12, dtype=np.float32)
        xy_errors, joint_squared_errors, height_errors, gravity_z_errors = [], [], [], []
        frame_count = len(goal_features_by_frame)
        failed = False
        for frame in range(frame_count):
            quaternion_conjugate = simulation.qpos[root_qpos[3:]] * conjugate_sign
            mujoco.mju_rotVecQuat(projected_gravity, gravity, quaternion_conjugate)
            observation = (
                simulation.qpos[joint_qpos] - home,
                simulation.qvel[joint_qvel],
                simulation.qvel[root_qvel[3:]],
                projected_gravity.copy(),
                action,
            )
            history = history[1:] + [observation] if history else [observation] * 10
            obs_input = np.concatenate([np.concatenate(component) for component in zip(*history)]).astype(np.float32)
            goal_input = goal_features_by_frame[np.minimum(frame + np.arange(10), frame_count - 1)].ravel()
            with torch.inference_mode():
                action = policy(torch.from_numpy(obs_input[None]), torch.from_numpy(goal_input[None]))[0].numpy()
            simulation.ctrl[actuators] = home + 0.25 * action
            mujoco.mj_step(model, simulation, nstep=4)

            quaternion_conjugate = simulation.qpos[root_qpos[3:]] * conjugate_sign
            mujoco.mju_rotVecQuat(projected_gravity, gravity, quaternion_conjugate)
            xy_errors.append(np.linalg.norm(simulation.qpos[root_qpos[:2]] - reference_root_position[frame, :2]))
            joint_squared_errors.append(np.square(simulation.qpos[joint_qpos] - reference_joint_position[frame]).mean())
            height_errors.append(abs(simulation.qpos[root_qpos[2]] - reference_root_position[frame, 2]))
            gravity_z_errors.append(abs(projected_gravity[2] - goal_features_by_frame[frame, 26]))
            if not np.isfinite(simulation.qpos).all() or height_errors[-1] > 0.085 or gravity_z_errors[-1] > 0.5:
                failed = True
                break
        results.append({
            "clip": path.stem,
            "frames": frame_count,
            "steps": len(xy_errors),
            "failed": failed,
            "mean_xy_drift_m": float(np.mean(xy_errors)),
            "joint_rmse_rad": float(np.sqrt(np.mean(joint_squared_errors))),
            "max_height_error_m": float(max(height_errors)),
            "max_gravity_z_error": float(max(gravity_z_errors)),
        })
    return results


def main():
    parser = argparse.ArgumentParser(description="Reproduce the original dog-v2 training benchmarks.")
    parser.add_argument("training_repo", type=Path, help="Original training repository with the dog-v2 checkpoint and motion library")
    parser.add_argument("mode", choices=("training", "native"))
    args = parser.parse_args()

    import torch
    from supermimic.eval.benchmark import load_policy_from_checkpoint

    torch.set_num_threads(2 if args.mode == "training" else 1)
    training_path = args.training_repo.resolve()
    motion_dir = training_path / "data/npz/dog"
    checkpoint = training_path / "logs/dog-v2/ckpt_latest.pt"
    with redirect_stdout(sys.stderr):
        policy, config = load_policy_from_checkpoint(checkpoint, "cpu")
        results = (
            benchmark_training(policy, config, motion_dir)
            if args.mode == "training"
            else benchmark_native(policy, motion_dir)
        )
    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
