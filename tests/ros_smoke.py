"""在 ROS 2 Humble 容器内验证仿真服务和话题。"""

import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from dog_controller.config import JOINT_NAMES
from dog_policy.bundle import PERIOD_NS


DURATION = 100 * PERIOD_NS


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def wait_until(node, condition, process, timeout=10):
    deadline = time.monotonic() + timeout
    while not condition():
        if process.poll() is not None:
            raise RuntimeError(f"Simulation exited early with status {process.returncode}")
        if time.monotonic() >= deadline:
            raise TimeoutError("Timed out waiting for the ROS smoke test condition")
        rclpy.spin_once(node, timeout_sec=0.05)


def call(node, client, process):
    future = client.call_async(Trigger.Request())
    wait_until(node, future.done, process)
    response = future.result()
    if not response.success:
        raise RuntimeError(response.message)


def main():
    process = subprocess.Popen([
        sys.executable, "-c", "from dog_sim.node import main; raise SystemExit(main())",
        "--ros-args", "-r", "__node:=dog_sim_smoke", "-r", "joint_states:=/dog_smoke/joint_states",
        "-r", "/clock:=/dog_smoke/clock", "-p", "policy_version:=dog-v2", "-p", "viewer:=false",
    ])
    rclpy.init()
    node = Node("dog_sim_smoke_client")
    clocks = []
    states = []
    node.create_subscription(Clock, "/dog_smoke/clock", clocks.append, 256)
    node.create_subscription(JointState, "/dog_smoke/joint_states", states.append, 256)
    play = node.create_client(Trigger, "/dog_sim_smoke/play")
    stop = node.create_client(Trigger, "/dog_sim_smoke/stop")
    try:
        wait_until(node, lambda: play.service_is_ready() and stop.service_is_ready()
                   and node.count_publishers("/dog_smoke/clock")
                   and node.count_publishers("/dog_smoke/joint_states"), process)
        call(node, play, process)
        wait_until(node, lambda: clocks and stamp_ns(clocks[-1].clock) >= 3 * PERIOD_NS, process)
        call(node, stop, process)

        # 等待已发出的消息送达，再验证暂停期间时钟冻结。
        for _ in range(8):
            rclpy.spin_once(node, timeout_sec=0.05)
        paused = stamp_ns(clocks[-1].clock)
        for _ in range(4):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert stamp_ns(clocks[-1].clock) == paused, "Simulation time advanced while paused"

        call(node, play, process)
        expected_end = paused + DURATION
        wait_until(node, lambda: stamp_ns(clocks[-1].clock) == expected_end and states
                   and stamp_ns(states[-1].header.stamp) == expected_end, process)
        timestamps = [stamp_ns(message.clock) for message in clocks]
        assert timestamps == sorted(timestamps), "Simulation time went backwards"
        assert all(value % PERIOD_NS == 0 for value in timestamps), "Simulation time did not align with the policy period"
        assert all(
            tuple(state.name) == JOINT_NAMES and len(state.position) == len(state.velocity) == 12
            and all(map(math.isfinite, [*state.position, *state.velocity]))
            for state in states
        ), "Invalid joint feedback"
        assert max(abs(a - b) for a, b in zip(states[0].position, states[-1].position)) > 1e-3
        print("ROS smoke test passed: play, pause, replay, clock and joint feedback")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


if __name__ == "__main__":
    main()
