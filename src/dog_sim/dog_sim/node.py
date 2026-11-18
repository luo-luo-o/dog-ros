"""ROS 2 仿真入口。"""

import hashlib
import os
from pathlib import Path
from time import monotonic

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from dog_controller.config import JOINT_NAMES
from dog_policy.bundle import PERIOD_NS, Bundle, valid_name

from .backend import MujocoBackend
from .driving import DrivingEpisode
from .episode import Episode
from .keyboard import KEY_TIMEOUT, Keyboard
from .motion import Motion


PREVIEW_HOLD_SECONDS = 0.6


class SimulationNode(Node):
    def __init__(self):
        super().__init__("dog_sim")
        self.viewer = None
        self.keyboard = None
        self.command_time = float('-inf')
        self.loop_playback = False
        self.preview_repeat_at = 0.0
        for name, default in (
            ("policy_dir", os.environ.get("DOG_POLICY_DIR", "policies/deploy")),
            ("motion_dir", os.environ.get("DOG_MOTION_DIR", "motions/dog")),
            ("clip", ""),
            ("policy_version", ""),
            ("viewer", os.environ.get("DOG_SIM_VIEWER", "false") == "true"),
            ("preview", True),
            ("mode", "clip"),
            ("keyboard", True),
        ):
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        self.clock_publisher = self.create_publisher(ClockMessage, "/clock", 1)
        self.joint_publisher = self.create_publisher(JointState, "joint_states", 10)

    def configure(self):
        bundle = Bundle.load(
            Path(self.get_parameter("policy_dir").value),
            self.get_parameter("policy_version").value or None,
        )
        mode = self.get_parameter("mode").value
        if mode not in ("clip", "teleop"):
            raise ValueError("mode must be clip or teleop")
        self.driving = mode == "teleop"
        sim_share = Path(get_package_share_directory("dog_sim"))
        controller_share = Path(get_package_share_directory("dog_controller"))
        backend = MujocoBackend(sim_share / "model/dog.xml", controller_share / "model/dog/dog.urdf")
        if self.driving:
            self.episode = DrivingEpisode(backend, bundle)
        else:
            clip = valid_name(self.get_parameter("clip").value or bundle.clip)
            clip_path = Path(self.get_parameter("motion_dir").value) / f"{clip}.npz"
            if clip == bundle.clip and hashlib.sha256(clip_path.read_bytes()).hexdigest() != bundle.clip_sha256:
                raise ValueError("Default motion clip checksum mismatch")
            motion = Motion.load(clip_path, bundle.home)
            self.episode = Episode(backend, bundle, motion)
        self.preview_enabled = not self.driving and self.get_parameter("preview").value
        if self.get_parameter("viewer").value:
            from .visualization import SimulationViewer

            backend.reset(self.episode.motion)
            self.viewer = SimulationViewer(backend)
            self.loop_playback = self.preview_enabled
            if not self.driving:
                self.get_logger().info("MuJoCo viewer opened. Press Space to play, pause or replay.")
        self.create_service(Trigger, "~/play", self._play)
        self.create_service(Trigger, "~/stop", self._stop)
        self.create_timer(PERIOD_NS / 1e9, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        if self.driving:
            self.create_subscription(Twist, "cmd_vel", self._on_twist, 1)
            if self.get_parameter("keyboard").value:
                self.keyboard = Keyboard()
                self.get_logger().info(
                    "Focus the terminal: u i o / j k l / m , .   "
                    "Space or k: stop. q/z: speed; w/x: linear; e/c: turn. Ctrl+C: exit."
                )
            self._start_playback()
            self.get_logger().info(f"Policy {bundle.name} is ready for teleop")
        else:
            self.get_logger().info(f"Policy {bundle.name} is ready to play {clip} ({len(motion)} frames)")

    def _start_playback(self, *, loop=False):
        self.loop_playback = loop and self.viewer is not None
        self._publish(self.episode.play())
        self._sync_viewer()

    def _pause_playback(self):
        self.loop_playback = False
        self.episode.stop()

    def _play(self, request, response):
        try:
            self._start_playback()
        except Exception as error:
            self._pause_playback()
            response.message = f"Failed to start playback: {error}"
            self.get_logger().error(response.message)
        else:
            response.success = True
            response.message = "Playback started from the first frame"
        return response

    def _stop(self, request, response):
        self._pause_playback()
        response.success = True
        response.message = "Playback paused"
        return response

    def _on_twist(self, message):
        self._set_velocity(message.linear.x, message.angular.z, monotonic())

    def _set_velocity(self, linear, yaw, now):
        if self.episode.running:
            self.episode.set_velocity(linear, yaw)
            self.command_time = now

    def _poll_driving(self, stop_from_viewer):
        now = monotonic()
        keys = self.keyboard.read(now) if self.keyboard is not None else None
        if stop_from_viewer:
            self._set_velocity(0, 0, now)
        elif keys is not None:
            self._set_velocity(*keys, now)
        elif now - self.command_time > KEY_TIMEOUT:
            self.episode.set_velocity(0, 0)

    def _tick(self):
        toggle = self.viewer is not None and self.viewer.take_playback_toggle()
        if self.driving:
            self._poll_driving(toggle)
            toggle = False
        if toggle and (self.episode.running or self.loop_playback):
            self._pause_playback()
            self.get_logger().info("Playback paused")
            return
        repeat = self.loop_playback and not self.episode.running and monotonic() >= self.preview_repeat_at
        if toggle or repeat:
            try:
                self._start_playback(loop=self.preview_enabled)
            except Exception as error:
                self._pause_playback()
                self.get_logger().error(f"Failed to start playback: {error}")
            return
        if not self.episode.running:
            self._sync_viewer()
            return
        try:
            self._publish(self.episode.step())
        except Exception as error:
            self._pause_playback()
            self.get_logger().error(f"Playback interrupted: {error}")
            return
        self._sync_viewer()
        if not self.episode.running:
            if self.loop_playback:
                self.preview_repeat_at = monotonic() + PREVIEW_HOLD_SECONDS
            else:
                self.get_logger().info(f"Playback complete ({self.episode.frame} frames)")

    def _sync_viewer(self):
        if self.viewer is not None and not self.viewer.sync(self.episode.backend):
            self.close_viewer()
            self.get_logger().info("MuJoCo viewer closed")

    def close_viewer(self):
        self.loop_playback = False
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

    def _publish(self, observation):
        clock = ClockMessage()
        clock.clock.sec, clock.clock.nanosec = divmod(self.episode.time_ns, 1_000_000_000)
        self.clock_publisher.publish(clock)
        joints = JointState()
        joints.header.stamp = clock.clock
        joints.name = list(JOINT_NAMES)
        joints.position = list(map(float, observation.position))
        joints.velocity = list(map(float, observation.velocity))
        self.joint_publisher.publish(joints)


def main():
    rclpy.init()
    node = SimulationNode()
    status = 0
    try:
        node.configure()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        if rclpy.ok():
            node.get_logger().fatal(f"Simulation failed: {error}")
            status = 1
    finally:
        if node.keyboard is not None:
            node.keyboard.close()
        node.close_viewer()
        node.destroy_node()
        rclpy.try_shutdown()
    return status
