"""在仿真节点的交互式终端读取平面速度指令。"""

import os
import sys
import termios
import tty
from select import select

from .gait import MAX_SPEED, MAX_YAW


KEY_TIMEOUT = 0.65
MOVE = {
    "u": (1, 1), "i": (1, 0), "o": (1, -1),
    "j": (0, 1), "k": (0, 0), "l": (0, -1),
    "m": (-1, -1), ",": (-1, 0), ".": (-1, 1),
}
SCALE = {
    "q": (1.1, 1.1), "z": (0.9, 0.9),
    "w": (1.1, 1.0), "x": (0.9, 1.0),
    "e": (1.0, 1.1), "c": (1.0, 0.9),
}


class Keyboard:
    def __init__(self):
        if not sys.stdin.isatty():
            raise RuntimeError("Keyboard teleop needs an interactive terminal; use keyboard:=false for /cmd_vel")
        self.fd = sys.stdin.fileno()
        self.terminal = termios.tcgetattr(self.fd)
        self.speed = 0.14
        self.turn = 0.40
        self.direction = (0, 0)
        self.last_move = float("-inf")
        tty.setcbreak(self.fd)

    def read(self, now: float) -> tuple[float, float] | None:
        if not select([self.fd], [], [], 0)[0]:
            return None
        data = os.read(self.fd, 64)
        if not data:
            self.direction = (0, 0)
        for key in data.decode("ascii", errors="replace").lower():
            if key in SCALE:
                linear, yaw = SCALE[key]
                self.speed = min(MAX_SPEED, max(.03, self.speed * linear))
                self.turn = min(MAX_YAW, max(.10, self.turn * yaw))
                if now - self.last_move > KEY_TIMEOUT:
                    self.direction = (0, 0)
            else:
                self.direction = MOVE.get(key, (0, 0))
                self.last_move = now
        return self.direction[0] * self.speed, self.direction[1] * self.turn

    def close(self) -> None:
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.terminal)
