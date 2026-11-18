"""在独立的 MuJoCo 状态中显示 ROS 仿真。"""

import os
from copy import copy
from threading import Event

import glfw
import mujoco
from mujoco import viewer

from .backend import MujocoBackend


class SimulationViewer:
    def __init__(self, backend: MujocoBackend):
        if not os.environ.get("DISPLAY"):
            raise RuntimeError("The MuJoCo viewer requires DISPLAY and X11 authorization")
        if not glfw.init():
            raise RuntimeError("GLFW could not connect to X11; check DISPLAY and XAUTHORITY")
        try:
            # 先检查窗口和 OpenGL 上下文，避免被动窗口启动线程失败后阻塞。
            glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
            probe = glfw.create_window(1, 1, "MuJoCo OpenGL check", None, None)
            if not probe:
                raise RuntimeError("GLFW could not create an OpenGL window")
            glfw.destroy_window(probe)
        finally:
            glfw.terminate()

        # 显示器可以修改其副本的参数，但不能修改策略使用的物理状态。
        self.model = copy(backend.model)
        self.data = mujoco.MjData(self.model)
        for index in range(self.model.ngeom):
            name = self.model.geom(index).name
            if name.endswith("_foot_collision"):
                self.model.geom_rgba[index] = (0.20, 0.24, 0.27, 1)
            elif name.endswith("_collision"):
                self.model.geom_rgba[index, 3] = 0
            elif name.endswith("_visual"):
                self.model.geom_rgba[index] = (0.70, 0.77, 0.82, 1)
        self._copy_state(backend)
        self.playback_toggle = Event()
        self.handle = viewer.launch_passive(
            self.model, self.data, key_callback=self._on_key, show_left_ui=False, show_right_ui=False,
        )
        with self.handle.lock():
            self.handle.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.handle.cam.trackbodyid = backend.root
            self.handle.cam.distance = 0.92
            self.handle.cam.azimuth = 115
            self.handle.cam.elevation = -26
        self.handle.sync()

    def _on_key(self, key: int) -> None:
        if key == glfw.KEY_SPACE:
            self.playback_toggle.set()

    def take_playback_toggle(self) -> bool:
        if not self.handle.is_running() or not self.playback_toggle.is_set():
            return False
        self.playback_toggle.clear()
        return True

    def _copy_state(self, backend: MujocoBackend) -> None:
        self.data.qpos[:] = backend.data.qpos
        self.data.qvel[:] = backend.data.qvel
        self.data.time = backend.data.time

    def sync(self, backend: MujocoBackend) -> bool:
        if not self.handle.is_running():
            return False
        with self.handle.lock():
            self._copy_state(backend)
        self.handle.sync(state_only=True)
        return True

    def close(self) -> None:
        self.handle.close()
