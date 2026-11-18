"""共享模型约定。"""

LEGS = ("FL", "FR", "RL", "RR")
JOINT_NAMES = tuple(
    f"{leg}_{part}_joint" for leg in LEGS for part in ("hip", "thigh", "calf")
)
BODY_FRAME = "trunk"
