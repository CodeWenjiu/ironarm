"""MuJoCo UR5e arm model path and helpers."""

import os

MODEL_PATH = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "ironarm_model", "ur5e.xml"
)

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

EE_BODY = "wrist_3_link"
