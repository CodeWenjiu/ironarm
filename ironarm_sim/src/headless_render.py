"""Headless render test — 启动 Copper 流水线，等待稳定，输出末端误差。

Usage:
    cd ironarm_sim && uv run python src/headless_render.py
"""

from __future__ import annotations

import sys
import time

import ironarm_sim
import mujoco
import numpy as np

from arm.model import JOINT_NAMES, MODEL_PATH


def main() -> int:
    # 1. 加载模型
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    # 2. 启动 Copper
    print("启动 Copper 流水线...")
    ironarm_sim.start_copper()

    # 3. 等待稳定
    print("等待稳定 (3s)...")
    start = time.time()
    last_state = None
    while time.time() - start < 3.0:
        state = ironarm_sim.poll_state()
        if state is not None:
            last_state = state
        time.sleep(0.01)

    if last_state is None:
        print("失败: 未收到状态数据")
        return 1

    j0, j1, j2, j3, j4, j5, wx, wy, wz = last_state

    # 4. MuJoCo FK
    for i, name in enumerate(JOINT_NAMES):
        data.joint(name).qpos[0] = last_state[i]
    tid = model.body("target").id
    jid = model.body_jntadr[tid]
    data.qpos[jid : jid + 3] = (wx, wy, wz)
    mujoco.mj_forward(model, data)

    ee_pos = data.site("attachment_site").xpos.copy()
    target = np.array([wx, wy, wz])
    err_mm = float(np.linalg.norm(ee_pos - target)) * 1000.0

    # 5. 输出
    print(f"关节: {j0:.3f} {j1:.3f} {j2:.3f} {j3:.3f} {j4:.3f} {j5:.3f}")
    print(f"目标: {wx:.3f} {wy:.3f} {wz:.3f}")
    print(f"末端: {ee_pos[0]:.3f} {ee_pos[1]:.3f} {ee_pos[2]:.3f}")
    print(f"误差: {err_mm:.1f} mm")

    if err_mm < 50.0:
        print("通过")
        return 0
    else:
        print(f"失败: 误差 {err_mm:.0f}mm >= 50mm")
        return 1


if __name__ == "__main__":
    sys.exit(main())
