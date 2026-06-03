"""无界面测试——通过终端输出验证 Copper 流水线是否正常运行。

启动 Copper，定时拉取共享内存中的关节状态和目标位置，
以表格形式输出到终端。
"""

import sys
import time

import ironarm_sim

HEADER = f"{'t':>6s} {'j0':>8s} {'j1':>8s} {'j2':>8s} {'j3':>8s} {'wx':>8s} {'wy':>8s} {'wz':>8s}"
SEP = "-" * len(HEADER)


def main() -> int:
    print("启动 Copper 运行时（无界面，轮询模式）...")
    print(HEADER)
    print(SEP)

    ironarm_sim.start_copper()

    start = time.time()
    count = 0
    last_print = start

    deadline = time.time() + 6.0
    while time.time() < deadline:
        state = ironarm_sim.poll_state()
        if state is not None:
            count += 1
            now = time.time()
            if now - last_print >= 0.2:
                last_print = now
                elapsed = now - start
                j0, j1, j2, j3, wx, wy, wz = state
                print(
                    f"{elapsed:5.1f}s {j0:+7.3f} {j1:+7.3f} {j2:+7.3f} {j3:+7.3f} {wx:+7.3f} {wy:+7.3f} {wz:+7.3f}"
                )
        else:
            time.sleep(0.001)

    print(SEP)
    print(f"结果: {count} 条状态 / ~6s")
    if count == 0:
        print("失败: 环形缓冲中无数据")
        return 1
    print("通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
