"""MuJoCo UR5e 模型路径和关节名称常量。"""

import os

# 模型文件路径（相对于本文件向上三级到仓库根目录）
MODEL_PATH = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "ironarm_model", "ur5e.xml"
)

# MuJoCo 关节名称（与 ur5e.xml 中的 name 属性一致）
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# 末端执行器 body 名称
EE_BODY = "wrist_3_link"
