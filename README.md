# 机器狗 ROS 2 控制

## 仿真

在 Linux 桌面或 Windows 的 WSL2 发行版终端进入工作空间，执行下面的命令。Linux 桌面需要 X11 或 XWayland、`xauth` 和 Docker Compose。Windows 环境见下方说明。

```bash
./docker/sim.sh
```

脚本为当前桌面配置 X11 并复用已有容器，镜像只在本地缺失时构建。打开窗口后，`dog-v2` 策略自动循环播放约 2 秒的默认转向步。每轮结束短暂停留，再回到首帧。点击窗口后按 **空格**暂停，再按空格从首帧重播。回到终端按 Ctrl+C 可退出仿真节点，开发容器继续运行。

需要交互式 Bash 时运行 `./docker/sim.sh bash`。修改源码后，用 `./docker/sim.sh --build bash` 在同一容器内更新 ROS 安装产物。从 Bash 手动启动仿真可用下面的命令。

```bash
ros2 run dog_sim simulate
ros2 run dog_sim simulate --ros-args -p viewer:=false
```

要开窗后等待手动播放，在启动命令末尾加 `--ros-args -p preview:=false`。ROS 的 `/dog_sim/play` 服务执行单次播放。

没有桌面会话时，用 `DOG_SIM_VIEWER=false ./docker/sim.sh bash` 进入无窗口环境，或用 `DOG_SIM_VIEWER=false ./docker/sim.sh` 直接启动无窗口节点。ROS 服务与话题见 [架构说明](docs/architecture.md)。

### Windows 上的 WSL2 和 WSLg

在 PowerShell 中用 `wsl.exe --list --verbose` 确认发行版运行在 WSL2。图形模式使用 WSL 自带的 WSLg，可用 `wsl.exe --update` 更新。Docker 可以使用发行版内的 Docker Engine 和 Compose 插件，也可以使用开启了该发行版 WSL 集成的 Docker Desktop（WSL2 后端）。先在发行版中确认 `docker version` 和 `docker compose version` 可用。[WSLg 安装说明](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps) · [Docker Desktop WSL2 设置](https://docs.docker.com/desktop/features/wsl/)

以 `E:\codes-GitHub\dog-ros` 为例，从 WSL2 的 Bash 启动仿真。

```bash
cd /mnt/e/codes-GitHub/dog-ros
./docker/sim.sh
```

也可以在仓库目录下的 PowerShell 中运行 `wsl.exe -d Ubuntu -- ./docker/sim.sh`，将 `Ubuntu` 替换为实际发行版名称。脚本使用 WSLg 设置的 `DISPLAY` 和 `/mnt/wslg/.X11-unix` 的 X11 套接字，MuJoCo 用软件 OpenGL 渲染。已有容器缺少对应的套接字挂载时，Compose 会应用新挂载，ROS 构建产物仍保存在仿真命名卷中。[WSLg 容器显示说明](https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md)

### 键盘遥控

首次更新源码时，在工作空间根目录运行以下命令。它在原容器内部构建 ROS 包，再打开遥控仿真。

```bash
./docker/sim.sh --build ros2 run dog_sim simulate --ros-args -p mode:=teleop
```

后续启动时可去掉 `--build`。保持焦点在启动仿真的**终端**并按住方向键对应的字母，MuJoCo 窗口显示实际运动。按键布局参考 [teleop_twist_keyboard](https://github.com/ros2/teleop_twist_keyboard)。

```text
u  i  o
j  k  l
m  ,  .
```

- `i` 前进，`,` 后退，`j / l` 左右转，四角键边走边转。
- `k` 或空格回到站立。松键或命令中断约 0.65 秒后自动停步。
- `q / z` 调整整体速度，`w / x` 调整线速度，`e / c` 调整转速。Ctrl+C 退出仿真节点。

直行或后退时，转向命令绝对值不超过 0.05 rad/s 会锁定当前航向。超过死区可手动转向，回到死区后锁定新朝向。停步会释放航向锁定。

遥控模式实时生成步态参考，`dog-v2` 的修正限幅后作用于实际关节目标。需要使用 ROS 遥控节点时，加 `-p keyboard:=false`，持续向 `/cmd_vel` 发布 `geometry_msgs/msg/Twist` 的 `linear.x` 和 `angular.z`。

### 其他动作

ROS 工作空间自带默认片段。要在已有容器里试播其他片段，可把选中的单个 NPZ 放到已挂载的开发区。完整动作库仍留在训练仓库。

```bash
clip=ai4a_D1_004_KAN01_001_s0
install -D "$HOME/workspace/supermimic-quadruped/data/npz/dog/$clip.npz" ".docker-runtime/motions/$clip.npz"
./docker/sim.sh ros2 run dog_sim simulate --ros-args \
  -p motion_dir:=/workspace/.docker-runtime/motions -p clip:="$clip"
```

## 节点和安装目录

- 仿真节点 `/dog_sim` 的入口是 [node.py](src/dog_sim/dog_sim/node.py)。策略、控制器和 MuJoCo 在同一进程内运行。
- 实物节点 `/dog_hardware` 的入口是 [node.py](src/dog_hardware/dog_hardware/node.py)。实物驱动等待 IK 标定和舵机协议。
- 源码挂载到 `/workspace/src`。两个服务通过各自的 Docker 命名卷，将 `build`、`install` 和 `log` 保留在容器内的 `/opt/dog`。仿真可执行文件一般位于 `/opt/dog/install/lib/dog_sim/simulate`。复用没有构建卷的旧容器时，产物保存于已挂载工作空间的 `.docker-runtime/colcon`。可在容器内用 `ros2 pkg prefix dog_sim` 查看实际安装前缀。

## 策略与验收

在仿真容器的 Bash 中管理策略或执行回归。

```bash
ros2 run dog_policy policy list
ros2 run dog_policy policy check dog-v2
ros2 run dog_policy policy activate dog-v2
python3 -m unittest discover -s tests -v
python3 tests/ros_smoke.py
```

策略和 `manifest.json` 位于 `policies/deploy/<version>/`。切换后重启仿真节点。训练侧导出、完整动作库与评测见 [实验报告](docs/experiments.md)。

实物检查从宿主机运行以下命令，等构建完成后进入容器。

```bash
docker compose up -d --no-recreate hardware
docker compose logs --tail 30 hardware
docker compose exec hardware dog-ros-entrypoint bash
```

在容器内执行 `ros2 run dog_policy policy check` 或 `ros2 run dog_hardware hardware`。实物 IK 和舵机待办见 [TODO](TODO)。
