# dog-v2 策略与动作片段评测

## 结论

默认步行片段选用 `ai4a_D1_ex03_KAN02_012_s20.npz`。它在 62 条留出片段中的步行类别回报最高，完整跟踪 100 帧（2.00 秒），并通过原生 MuJoCo 复核。片段作为策略的运行时参考输入，导出的策略仍接受观测历史和任意符合约定的目标窗口。

## 训练场景闭环

在 2026-11-18 使用原训练仓库的 `run_benchmark` 函数，在 CPU 上评测清单中 `keep=1` 的全部 462 条动作。第一批为留出集，第二批为训练与困难集。每批包含 48 个并行环境，随机种子为 0，关闭观测噪声、动作噪声、重置噪声和域随机化。每个片段从首帧参考状态开始。两批分别耗时约 38.16 秒和 213.14 秒，计时包含各批环境的创建和关闭。

| 分组 | 片段数 | 参考帧数 | 失败数 | 平均回报 | 按帧加权回报 | 平均平面漂移（米） | 平均偏航漂移（弧度） |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 全部 | 462 | 130,416 | 0 | 0.88859 | 0.90026 | 0.07485 | 0.07934 |
| 留出集 `holdout` | 62 | 13,056 | 0 | 0.86199 | 0.86902 | 0.07884 | 0.08684 |
| 训练集 `train` | 314 | 92,198 | 0 | 0.91721 | 0.93694 | 0.03487 | 0.06229 |
| 困难集 `hard` | 86 | 25,162 | 0 | 0.80328 | 0.78210 | 0.21926 | 0.13794 |

每条片段的归一化回报是累计回报除以「单步理论最大回报 × 参考帧数」。总体平均回报对片段等权，按帧加权回报为累计回报总和除以全库的理论最大回报。漂移指标对实际评测步数加权。训练环境按原有规则处理片段结束和异常终止。

将留出集限制在 `walk` 后，先排除失败或未完成的片段，再按归一化回报从高到低排序。分数相同时以平均平面漂移和名称排序。推荐片段得分 0.95839，平均平面漂移 0.01291 米，平均偏航漂移 0.02030 弧度。第二名为同一片段的镜像，得分 0.95489。留出集全类别第一名是静止片段 `ai4a_D1_013_KAN01_001_s0_mirror`，得分 0.98713。训练集步行第一名是 `ai4a_D1_009_KAN01_001_s0_mirror`，得分 0.97184。

逐片段数据包含各回报项的每步均值、跟踪漂移和终止信息。[训练场景汇总](experiment_data/training_summary.json) · [逐片段 CSV](experiment_data/training_clips.csv) · [逐片段 JSON](experiment_data/training_clips.json)

## 原生 MuJoCo 复核

使用原训练场景的编译模型和相同的 PyTorch 策略，以原生 MuJoCo CPU 步进复核五个候选。仿真从首帧参考姿态和速度初始化，控制周期为 20 毫秒，每次施加关节目标后推进四个 5 毫秒物理步。关节均方根误差均匀统计每一帧的 12 个关节。平面误差为每步仿真根位置与该步参考根位置的欧氏距离。

| 候选 | 运行帧数 / 参考帧数 | 平均平面漂移（米） | 关节均方根误差（弧度） | 最大高度误差（米） |
| --- | ---: | ---: | ---: | ---: |
| `ai4a_D1_ex03_KAN02_012_s20` | 100 / 100 | 0.01940 | 0.05165 | 0.00667 |
| `ai4a_D1_ex03_KAN02_012_s20_mirror` | 100 / 100 | 0.01888 | 0.05477 | 0.00535 |
| `tencent_dog_hit_001_s2` | 101 / 101 | 0.00859 | 0.08503 | 0.01026 |
| `ai4a_D1_013_KAN01_001_s0_mirror` | 253 / 253 | 0.01082 | 0.04693 | 0.00871 |
| `ai4a_D1_004_KAN01_001_s0` | 191 / 191 | 0.05768 | 0.06155 | 0.00787 |

两个实验均在原始训练资产和 CPU 推理环境下执行。原生复核使用 MuJoCo 3.16.0。[原生仿真候选 CSV](experiment_data/native_candidates.csv)

## 迁移后的闭环

部署链路通过当前工作空间的 `dog_policy`、`dog_controller` 与 `dog_sim` 直接执行，输入为工作空间自带的 ONNX 与 NPZ。该评测以原生 MuJoCo 步进，记录片段完成状态、轨迹误差和异常终止步数。ROS 服务和话题的传输行为另行验证。

在本地 CPU 环境用 MuJoCo 3.16.0、ONNX Runtime 1.31.1 和 NumPy 2.5.4 逐条执行全部片段，耗时 109.50 秒，共完成 130,416 个控制步。

| 分组 | 完成数 / 片段数 | 失败数 | 平均平面漂移（米） | 平均偏航漂移（弧度） | 关节均方根误差（弧度） |
| --- | ---: | ---: | ---: | ---: | ---: |
| 全部 | 462 / 462 | 0 | 0.07438 | 0.07843 | 0.09465 |
| 留出集 `holdout` | 62 / 62 | 0 | 0.07340 | 0.08376 | 0.09414 |
| 训练集 `train` | 314 / 314 | 0 | 0.03512 | 0.06145 | 0.07435 |
| 困难集 `hard` | 86 / 86 | 0 | 0.21874 | 0.13790 | 0.14691 |

误差按控制步汇总。默认片段完整运行 100 帧，平均平面漂移 0.01940 米，平均偏航漂移 0.02342 弧度，关节均方根误差 0.05165 弧度，最大高度误差 0.00667 米。本轮使用本地 Python 依赖版本，容器固定版本以镜像构建定义为准。[逐片段 CSV](experiment_data/workspace_clips.csv) · [全量 JSON 与模型哈希](experiment_data/workspace_benchmark.json)

当前工作空间随附默认片段。评测脚本默认读取随附清单。复现完整库评测时，可把训练仓库的动作目录作为输入。

```bash
training_repo="$HOME/workspace/supermimic-quadruped"
docker compose run --rm -T \
  -v "$training_repo/data/npz/dog:/reference_motions:ro" sim \
  python3 docs/experiment_data/benchmark_workspace.py --motion-dir /reference_motions
```

已有本地 Python 依赖时，可从工作空间根目录使用本地源码和训练动作库。

```bash
PYTHONPATH=src/dog_controller:src/dog_policy:src/dog_sim \
  python3 docs/experiment_data/benchmark_workspace.py \
  --motion-dir "$HOME/workspace/supermimic-quadruped/data/npz/dog"
```

## Docker Compose 容器验收

2026-11-18 使用 Docker Engine 29.10.0 和 Compose v5.8.0 构建两个镜像。仿真容器运行 Python 3.10.12、MuJoCo 3.3.7、ONNX Runtime 1.20.1 和 NumPy 1.26.4。六项回归和 ROS 冒烟测试通过，将工作空间复制到临时目录后，ROS 冒烟测试再次通过。实物容器校验策略并启动占位节点，通过了仿真模块和资产排除检查。

```bash
docker compose config --quiet
docker compose build sim hardware
docker compose run --rm -T -e ROS_DOMAIN_ID=73 sim bash -ec 'python3 -m unittest discover -s tests -v && python3 tests/ros_smoke.py'
docker compose run --rm -T hardware ros2 run dog_policy policy check
```

在 Compose 仿真容器中运行全量评测，耗时 113.01 秒。新构建的镜像单独复测了默认片段和两条失败片段，逐片段指标与全量结果一致。

| 分组 | 完成数 / 片段数 | 已运行步数 / 参考帧数 |
| --- | ---: | ---: |
| 全部 | 460 / 462 | 129,929 / 130,416 |
| 留出集 | 62 / 62 | 13,056 / 13,056 |
| 训练集 | 313 / 314 | 91,959 / 92,198 |
| 困难集 | 85 / 86 | 24,914 / 25,162 |

默认片段运行 100 / 100 帧，平均平面漂移 0.01569 米，关节均方根误差 0.05235 弧度。两条动作均在第 47 帧超出投影重力 Z 分量误差阈值 0.50。训练集的 `ai4a_D1_ex01_KAN01_001_s8` 达到 0.52943，困难集的 `tencent_dog_fast_run_02_004_s27` 达到 0.56269。此前本地评测在另一组依赖版本下完成了两条动作。保护阈值沿用原配置。[容器逐片段数据与资产哈希](experiment_data/compose_benchmark.json)

```bash
docker compose run --rm -T sim python3 docs/experiment_data/benchmark_workspace.py \
  --policy-version dog-v2 --clip ai4a_D1_ex03_KAN02_012_s20
```

### WSL2 与 WSLg 仿真验收

2026-11-22 在 Windows 的 E 盘工作区使用 Ubuntu 24.04.5（WSL2）、WSLg 1.0.71、Docker Engine 29.8.2 和 Compose v5.6.0。MuJoCo 3.3.7 通过 WSLg 和软件 OpenGL 成功开窗。先无窗口启动再开窗时复用了同一容器。用占位目录模拟旧容器的 X11 挂载后，Compose 更新了挂载并保留构建卷。10 项单元测试和 ROS 冒烟测试通过。Docker Desktop 与原生 Linux 桌面待实机验证，Linux 的 X11 凭据转发路径已在 WSL2 中验证。

```bash
docker compose config --quiet
docker compose build sim
./docker/sim.sh timeout --preserve-status -s INT 8s ros2 run dog_sim simulate
env -u DISPLAY DOG_SIM_VIEWER=false ./docker/sim.sh bash -ec \
  'python3 -m unittest discover -s tests -v && python3 tests/ros_smoke.py'
env -u DISPLAY DOG_SIM_VIEWER=false ./docker/sim.sh \
  python3 docs/experiment_data/benchmark_workspace.py \
  --policy-version dog-v2 --clip ai4a_D1_ex03_KAN02_012_s20
```

默认片段完成 100 / 100 帧，平均平面漂移 0.01569 米，关节均方根误差 0.05235 弧度。

| 运行资产 | SHA-256 |
| --- | --- |
| `policy.onnx` | `911a0ef53585a138ecb63308061f9eb591076310809af21910380ed4d1e2f42e` |
| `dog.xml` | `925ff6cd6e7a75cb45fd369c45f970ef687056976c13fe598303b1a68e4bb1ed` |
| `dog.urdf` | `ef7b0e3c3c6500ffe3bc3707caf3122a9068dd0e879f24dbb0090d4c9f2fbd9f` |
| `ai4a_D1_ex03_KAN02_012_s20.npz` | `001a26d9f1e8a19c536f2893a7867d43f94aeee2e5f32843d4b2d741764ea1b1` |

## 默认片段的显示复核

默认参考片段的躯干在 100 帧内移动约 0.097 米，偏航变化约 55°。这是一段短转向步，循环时会回到起始位置和姿态。窗口加入了棋盘地面、灯光和可见的足底接触球，每轮结束停留 0.6 秒。复测默认片段时，100 帧及所有跟踪指标和 [容器历史结果](experiment_data/compose_benchmark.json) 一致。

## 键盘与速度遥控

使用同一 `dog-v2` ONNX 和 MuJoCo 3.3.7，对站立后的不同命令分别运行 5 秒，再下发零速度。方向以开始施加命令时的机体朝向为参照。以下为实测位移和角度。

| 线速度命令（米/秒） | 转速命令（弧度/秒） | 前向位移（米） | 偏航变化（弧度） |
| ---: | ---: | ---: | ---: |
| +0.14 | 0 | +0.28 | +0.03 |
| -0.14 | 0 | -0.61 | +0.03 |
| 0 | +0.40 | -0.08 | +1.56 |
| 0 | -0.40 | -0.15 | -1.43 |

直接输出策略关节目标时，合成参考会使躯干在站立时降到约 0.14 米，高速前进也触发了姿态或高度保护。遥控改用 IK 基准与限幅策略修正后，上述测试的最低躯干高度约 0.21 米，均持续运行到结束。转向仍有平移，速度指令用于生成步态参考。

容器的图形模式也验证了终端按键触发关节运动、键盘超时回到站立，以及 `/cmd_vel` 接收转向与停止命令。单元测试覆盖方向、连续步进、停止及无效速度输入。

```bash
./docker/sim.sh --build bash
python3 -m unittest discover -s tests -p 'test_teleop.py' -v
```

## 遥控直行偏航诊断

在 MuJoCo 3.3.7 中，站立稳定 1 秒后下发 10 秒前进命令。分别测试 0.14 和 0.22 米/秒，每组使用五种起步相位，保留一个按默认时钟推进的相位。消融分别关闭 ONNX 修正、镜像右侧碰撞几何和惯量、使左右足端参考站姿对称，并单独试加航向反馈。以下位移和偏航都相对发令时的初始朝向，负值表示向右。

| 0.22 米/秒命令下的对照 | 10 秒平均前进（米） | 平均横向位移（米） | 平均偏航（度） |
| --- | ---: | ---: | ---: |
| 航向保持合入前的遥控 | 1.162 | -0.266 | -25.0 |
| 只保留 IK，关闭策略修正 | 1.536 | -0.111 | -8.8 |
| 镜像左右物理模型，保留策略修正 | 1.153 | -0.278 | -25.6 |
| 对称参考站姿，保留策略修正 | 1.160 | -0.267 | -25.4 |
| 同时对称物理模型和参考站姿，保留策略修正 | 1.159 | -0.278 | -25.8 |
| 当前遥控加航向反馈 | 1.186 | -0.002 | -0.6 |

航向保持合入前，遥控在 0.22 米/秒档五次都向右偏，横向 20.6 至 31.8 厘米。关闭策略修正后，有的起步相位仍右偏，也有相位左偏。合入前的 0.14 米/秒档没有固定偏向，横向终点从右偏 6.9 厘米到左偏 4.9 厘米。100 次试验均完成。

仿真实际加载 MJCF。URDF 在后端用于核对关节名、轴和限位，以及定位网格资源。策略的两条后腿 `HOME` 大腿和小腿角度各差约 2.38°，带来约 3.29 毫米的前后足端位置差；将该站姿对称化后高速右偏仍然存在。

对 30 个合成步态观测及参考窗口做左右镜像，用同一 ONNX 推理，再将输出镜像回原坐标。两次的绝对关节目标平均相差 0.031 弧度，最大相差 0.108 弧度。这说明当前合成步态上的策略输出存在左右不对称。实验没有对训练数据、奖励或权重版本做消融，因而无法归因到某一项训练设置。

诊断时原遥控前进只将偏航速度参考置零。实验加入航向反馈后，高速档五次横向偏差的绝对值均在 3.1 厘米以内。表中的反馈和模型对称化属于诊断对照。[原始数据与资产哈希](experiment_data/teleop_drift.json)

在仿真容器的 Bash 中复现。

```bash
python3 docs/experiment_data/analyze_teleop_drift.py
```

## 遥控航向保持复测

航向保持合入后，用同一 ONNX 和 MuJoCo 3.3.7，按五种起步相位持续下发 0.22 米/秒前进命令，同时每步交替输入 ±0.03 弧度/秒的转向噪声。转向死区为 ±0.05 弧度/秒，航向误差比例增益为 1.8。每次运行 500 步（10 秒），横向终点偏差最大 3.10 厘米，航向终点误差最大 1.43°。验证了手动转向释放航向目标、回到死区捕获新目标及停步清除锁定。默认片段完成 100 帧，跟踪指标与历史记录一致。[逐次数据与资产哈希](experiment_data/teleop_heading_hold.json)

```bash
python3 -m unittest discover -s tests -p 'test_teleop.py' -v
python3 tests/ros_smoke.py
```

## 实验环境与复现

训练仓库提交为 `1a19fb45bda12a0f7e1f9593b8731ad6a3a0d9c9`。评测时训练仓库的本地 `dog.xml` 把躯干的初始高度由 0 米改成了 0.31 米，复现以表中的实际文件哈希为准。评测使用 Intel Core i7-14700HX、Python 3.12.13、PyTorch 2.14.1+cu130、MuJoCo 3.16.0 和 Warp 1.18.0，推理和物理均运行在 CPU。

| 训练仓库中的输入 | SHA-256 |
| --- | --- |
| `logs/dog-v2/ckpt_latest.pt` | `a352db94c20d743731feb872b80f3f106b6ae0db3b74acb02d8310b316e3a087` |
| `data/npz/dog/manifest.csv` | `1f051c694d18d9497fea506da6594342bebb24d9de6fd088ee4cd8bd9aa9b0c5` |
| `data/npz/dog/ai4a_D1_ex03_KAN02_012_s20.npz` | `001a26d9f1e8a19c536f2893a7867d43f94aeee2e5f32843d4b2d741764ea1b1` |
| `src/supermimic/robots/assets/dog/dog.xml` | `416db9c22b7df22ee0579455df71cbdbe457f261824343aaa57d816c6797f90a` |

原始 checkpoint 和训练配置保存在训练仓库。工作空间随附的 `motions/dog/ai4a_D1_ex03_KAN02_012_s20.npz` 与表中源文件哈希一致。推理使用 `policies/deploy/dog-v2/policy.onnx`，其 SHA-256 为 `911a0ef53585a138ecb63308061f9eb591076310809af21910380ed4d1e2f42e`。

从当前工作空间的根目录运行以下命令。原训练仓库与完整片段库只用于复现本页历史实验。

```bash
training_repo="$HOME/workspace/supermimic-quadruped"
env PYTHONDONTWRITEBYTECODE=1 WARP_CACHE_PATH=/tmp OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  "$training_repo/.venv/bin/python" docs/experiment_data/reproduce_training.py "$training_repo" training
env PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  "$training_repo/.venv/bin/python" docs/experiment_data/reproduce_training.py "$training_repo" native
```

脚本在标准输出写入 UTF-8 JSON，进度日志发送到标准错误。按表中版本复跑后，训练场景的 462 条结果和原生仿真的 5 条结果与归档一致。执行前根据表格核对原始 checkpoint 和数据版本。对 CPU 或物理引擎版本不同的运行，应以重复实验实测值为准。[复现实验脚本](experiment_data/reproduce_training.py)

## 训练侧导出

从 ROS 工作空间根目录运行下面的命令。

```bash
ros_workspace="$PWD"
training_repo="$HOME/workspace/supermimic-quadruped"
(
  cd "$training_repo"
  .venv/bin/python "$ros_workspace/tools/export_initial_policy.py" "$PWD"
)
```

导出脚本逐帧对照 PyTorch 和 ONNX，将部署策略、策略清单、默认参考片段及回归样本写入训练仓库的 `out/dog-ros-export`。ROS 默认仿真使用当前工作空间内的部署产物。

## 资料来源

默认参考动作来自 AI4Animation 的狗动作捕捉数据，经 SuperMimic 训练仓库重定向到当前骨架。原仓库的 `THIRD_PARTY_NOTICES.md` 将该数据标为 CC BY-NC 4.0 并注明研究和教育用途，也记录了训练所用 Tencent 数据的研究用途限制。运动库与训练权重的来源和适用条款参见原仓库的 `THIRD_PARTY_NOTICES.md`、`configs/sources.yaml` 与 `LICENSE`。训练仓库只用于溯源和复现，迁移后的运行依赖由当前工作空间的镜像定义管理。
