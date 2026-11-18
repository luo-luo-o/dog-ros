# 工作空间说明

这是机器狗控制和仿真用的 ROS 2 Humble 工作空间。运行代码使用 Python，开发和依赖由 Docker Compose 管理。文件名、代码标识、命令、日志及机器可读字段使用英文。项目文档正文及代码注释使用中文 UTF-8。保留上游许可原文。

## 修改位置

- 在 `src/dog_controller` 维护控制协议、输入校验、URDF 及共享的 `config.py`。
- 在 `src/dog_policy` 维护 ONNX 推理、观测历史及策略版本命令。
- 在 `src/dog_sim` 维护 MJCF、参考片段、物理步进和仿真 ROS 节点。
- 在 `src/dog_hardware` 维护实物足端目标、IK 和驱动边界。接入设备前需取得舵机协议、标定、实物限位、反馈定义和控制频率。
- 修改模型时核对 URDF、MJCF 和策略中的 12 个关节名、轴、顺序及角度单位。策略周期 20 ms，实物周期待定义。策略动作历史保存原始输出。
- 模型、部署策略和默认参考片段须随工作空间迁移。完整动作库保留在训练仓库，用于训练、全量评测及导出。其他仿真片段可按需只读挂载。每次引入新的第三方依赖前，都要取得用户明确批准。

## 构建和验证

桌面开发用 `./docker/sim.sh` 打开默认的 X11 仿真窗口，无桌面环境用 `DOG_SIM_VIEWER=false ./docker/sim.sh bash`。

镜像构建用于安装依赖。项目文件由长驻 Compose 容器挂载，构建产物保存在命名卷中。仿真源码和模型变更后用 `./docker/sim.sh --build bash` 在现有容器内更新 ROS 包。常规开发复用容器和镜像。

```bash
docker compose config --quiet
./docker/sim.sh bash
```

在仿真容器内执行下面的命令。

```bash
python3 -m unittest discover -s tests -v
python3 tests/ros_smoke.py
```

修改策略、观测、时序、动力学或模型后，复跑默认片段与相应回归。记录命令、资产哈希、测试环境、指标及未验证项。[架构说明](docs/architecture.md) · [实验报告](docs/experiments.md) · [操作说明](README.md)
