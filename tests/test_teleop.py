"""验证 dog-v2 遥控模式的方向、连续步进与停止。"""

import unittest
from pathlib import Path

import numpy as np

from dog_policy.bundle import Bundle
from dog_sim.backend import MujocoBackend
from dog_sim.driving import DrivingEpisode
from dog_sim.gait import MAX_SPEED, MAX_YAW


ROOT = Path(__file__).resolve().parents[1]


def yaw(quaternion):
    w, x, y, z = quaternion
    return np.arctan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


class TeleopTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bundle = Bundle.load(ROOT / "policies/deploy", "dog-v2")

    def test_forward_reverse_turn_and_stop(self):
        for speed, turn in ((.14, 0), (-.14, 0), (0, .4), (0, -.4)):
            with self.subTest(speed=speed, turn=turn):
                backend = MujocoBackend(ROOT / "src/dog_sim/model/dog.xml",
                                       ROOT / "src/dog_controller/model/dog/dog.urdf")
                episode = DrivingEpisode(backend, self.bundle)
                episode.play()
                for _ in range(50):
                    episode.step()
                start = backend.root_position.copy()
                heading = yaw(backend.root_quaternion)
                episode.set_velocity(speed, turn)
                for _ in range(250):
                    episode.step()
                delta = backend.root_position[:2] - start[:2]
                if speed:
                    forward = delta @ [np.cos(heading), np.sin(heading)]
                    self.assertGreater(np.sign(speed) * forward, .16)
                else:
                    rotation = yaw(backend.root_quaternion) - heading
                    self.assertGreater(np.sign(turn) * rotation, 1.)
                self.assertEqual(episode.frame, 300)

                episode.set_velocity(0, 0)
                for _ in range(50):
                    episode.step()
                stopped = backend.root_position.copy()
                for _ in range(100):
                    episode.step()
                self.assertLess(np.linalg.norm(backend.root_position[:2] - stopped[:2]), .03)
                self.assertEqual(episode.time_ns, 9_000_000_000)
                self.assertTrue(episode.running)

    def test_velocity_limits_and_invalid_command(self):
        backend = MujocoBackend(ROOT / "src/dog_sim/model/dog.xml",
                               ROOT / "src/dog_controller/model/dog/dog.urdf")
        episode = DrivingEpisode(backend, self.bundle)
        episode.set_velocity(10, -10)
        np.testing.assert_array_equal(episode.gait.command, [MAX_SPEED, -MAX_YAW])
        episode.set_velocity(float("nan"), 0)
        np.testing.assert_array_equal(episode.gait.command, [0, 0])


if __name__ == "__main__":
    unittest.main()
