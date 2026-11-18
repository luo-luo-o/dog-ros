"""验证控制边界、策略输入及仿真行为。"""

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

import numpy as np

from dog_controller.config import JOINT_NAMES, LEGS
from dog_controller.controller import Controller
from dog_controller.interfaces import FootTargets, JointTargets, RobotObservation
from dog_hardware.backend import HardwareBackend
from dog_policy.bundle import ACTIVE, Bundle
from dog_policy.cli import activate
from dog_policy.tracker import Tracker
from dog_sim.backend import MujocoBackend
from dog_sim.episode import Episode
from dog_sim.motion import Motion


ROOT = Path(__file__).resolve().parents[1]
DEPLOYMENT = ROOT / "policies/deploy"
MARKER = "manifest.json"


class RecordingBackend:
    def write_joints(self, targets):
        self.received = targets

    def write_feet(self, targets):
        self.received = targets


class WorkspaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bundle = Bundle.load(DEPLOYMENT, "dog-v2")
        cls.clip = ROOT / "motions/dog" / f"{cls.bundle.clip}.npz"
        cls.motion = Motion.load(cls.clip, cls.bundle.home)

    def test_controller_reorders_and_rejects_invalid_targets(self):
        backend = RecordingBackend()
        controller = Controller(backend)
        controller.submit(JointTargets(JOINT_NAMES[::-1], tuple(range(12))))
        self.assertEqual(backend.received, JointTargets(JOINT_NAMES, tuple(range(11, -1, -1))))
        controller.submit(FootTargets(LEGS[::-1], ((0, 0, 0),) * 4))
        self.assertEqual(backend.received.legs, LEGS)
        for targets in (
            JointTargets((JOINT_NAMES[0],) * 12, (0,) * 12),
            JointTargets(JOINT_NAMES, (0,) * 11),
            JointTargets(JOINT_NAMES, (0,) * 11 + (float("nan"),)),
            FootTargets(LEGS, ((0, 0),) * 4),
        ):
            with self.assertRaises(ValueError):
                controller.submit(targets)
        with self.assertRaises(NotImplementedError):
            Controller(HardwareBackend()).read()

    def test_hardware_ik_boundary(self):
        ik = Mock()
        driver = Mock()
        controller = Controller(HardwareBackend(ik, driver))
        feet = FootTargets(LEGS, ((0., 0., -0.2),) * 4)
        ik.solve.return_value = JointTargets(JOINT_NAMES[::-1], (0.1,) * 12)
        controller.submit(feet)
        ik.solve.assert_called_once_with(feet)
        driver.write.assert_called_once_with(JointTargets(JOINT_NAMES, (0.1,) * 12))
        driver.write.reset_mock()
        ik.solve.return_value = JointTargets(JOINT_NAMES, (0.1,) * 11 + (float("nan"),))
        with self.assertRaises(ValueError):
            controller.submit(feet)
        driver.write.assert_not_called()

    def test_motion_validates_and_reorders(self):
        with np.load(self.clip, allow_pickle=False) as archive:
            original = dict(archive.items())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "clip.npz"
            np.savez(path, **{**original, "joint_names": original["joint_names"][::-1],
                              "joint_pos": original["joint_pos"][:, ::-1],
                              "joint_vel": original["joint_vel"][:, ::-1]})
            motion = Motion.load(path, self.bundle.home)
            np.testing.assert_array_equal(motion.features, self.motion.features)
            for changed in (
                {"fps": 25},
                {"joint_names": np.repeat(original["joint_names"][0], 12)},
                {"joint_names": original["joint_names"][:-1]},
                {"joint_pos": np.ones_like(original["joint_pos"]) * np.inf},
            ):
                np.savez(path, **{**original, **changed})
                with self.assertRaises(ValueError):
                    Motion.load(path, self.bundle.home)
            with self.assertRaises(FileNotFoundError):
                Motion.load(path.with_name("missing.npz"), self.bundle.home)

    def test_policy_agrees_with_training(self):
        with np.load(ROOT / "tests/fixtures/policy_reference.npz", allow_pickle=False) as expected:
            tracker = Tracker(self.bundle)
            for frame, goal in enumerate(expected["goal"]):
                ref_goal = self.motion.goal(frame)
                np.testing.assert_allclose(ref_goal[0], goal, atol=2e-6, rtol=1e-5)
                observation = RobotObservation(
                    *[tuple(expected[key][frame]) for key in ("position", "velocity", "angular_velocity", "gravity")]
                )
                raw_input = expected["obs_history"][frame:frame + 1]
                reference_action = expected["action"][frame]
                action = self.bundle.infer(raw_input, ref_goal)
                np.testing.assert_allclose(action, reference_action, atol=1e-5, rtol=1e-4)
                targets = tracker.predict(observation, ref_goal)
                expected_position = self.bundle.home + self.bundle.action_scale * reference_action
                np.testing.assert_allclose(targets.position, expected_position, atol=1e-5, rtol=1e-4)
                tracker.previous_action = reference_action

    def test_invalid_version_does_not_replace_active(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = DEPLOYMENT / self.bundle.name
            candidate = root / self.bundle.name
            candidate.mkdir()
            manifest = json.loads((source / MARKER).read_text(encoding="utf-8"))
            model = candidate / manifest["model"]
            model.write_bytes(b"invalid")
            (candidate / MARKER).write_text(json.dumps(manifest), encoding="utf-8")
            active = root / ACTIVE
            active.write_text("old-version\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "checksum mismatch"):
                activate(root, self.bundle.name)
            self.assertEqual(active.read_text(encoding="utf-8").strip(), "old-version")
            model.write_bytes((source / manifest["model"]).read_bytes())
            activate(root, self.bundle.name)
            self.assertEqual(active.read_text(encoding="utf-8").strip(), self.bundle.name)

    def test_simulation_completes_and_restarts(self):
        backend = MujocoBackend(ROOT / "src/dog_sim/model/dog.xml",
                               ROOT / "src/dog_controller/model/dog/dog.urdf")
        episode = Episode(backend, self.bundle, self.motion)
        start = episode.play()
        moved = episode.step()
        self.assertGreater(np.linalg.norm(np.subtract(start.position, moved.position)), 0.001)
        episode.stop()
        with self.assertRaises(RuntimeError):
            episode.step()
        old_time = episode.time_ns
        replay = episode.play()
        np.testing.assert_array_equal(start.position, replay.position)
        while episode.running:
            final = episode.step()
        self.assertEqual(episode.frame, 100)
        self.assertEqual(episode.time_ns - old_time, 2_000_000_000)
        self.assertEqual(len(final.position), 12)
        self.assertTrue(np.isfinite(final.position).all())


if __name__ == "__main__":
    unittest.main()
