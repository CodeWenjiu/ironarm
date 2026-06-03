"""应用主窗口——3D 机械臂可视化。

启动流程：
1. 打印 GPU 信息
2. 在独立线程启动 Copper DAG（运动规划 + IK + 插值 + 驱动）
3. 创建 Qt 窗口，内嵌 MuJoCo 渲染
4. Qt 主循环中定时拉取 Copper 状态 → 更新 MuJoCo 场景
"""

import atexit
import os
import sys

import ironarm_sim as _rust  # type: ignore[import-untyped]
from PySide6.QtCore import QTimer
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtWidgets import QApplication, QMainWindow

from .arm.view import Arm3DView


def _reset_terminal() -> None:
    """Copper TUI 退出后恢复终端设置。"""
    if sys.stdout.isatty():
        os.system("stty sane 2>/dev/null")


atexit.register(_reset_terminal)


def _print_gpu_info() -> None:
    """在 Copper TUI 接管终端前打印 GPU 信息。"""
    QApplication.instance() or QApplication(sys.argv)
    widget = QOpenGLWidget()
    widget.resize(1, 1)
    widget.show()
    widget.makeCurrent()
    gl = widget.context().functions()
    vendor = gl.glGetString(0x1F00)
    renderer = gl.glGetString(0x1F01)
    if vendor:
        v = vendor if isinstance(vendor, str) else vendor.decode()
        r = renderer if isinstance(renderer, str) else renderer.decode()
        print(f"GPU: {v} — {r}")
    else:
        print("GPU: (无 GL 上下文)")
    widget.doneCurrent()
    widget.hide()
    widget.destroy()


class MainWindow(QMainWindow):
    """主窗口——包含 MuJoCo 3D 视图。"""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("IronArm Simulator — UR5e")
        self._view3d = Arm3DView()
        self.setCentralWidget(self._view3d)
        self.resize(900, 550)

    def closeEvent(self, event) -> None:
        """关闭窗口时同时退出 Qt 事件循环。"""
        print("正在关闭...")
        QApplication.quit()
        super().closeEvent(event)


def main() -> None:
    """应用入口。"""
    _print_gpu_info()

    # 启动 Copper 运行时（独立线程）
    _rust.start_copper()

    app = QApplication.instance()
    window = MainWindow()
    window.show()

    # 监控 Copper 是否仍在运行
    def _watch_copper() -> None:
        if not _rust.is_copper_alive():
            print("Copper 已退出，关闭 GUI。")
            app.quit()

    timer = QTimer()
    timer.timeout.connect(_watch_copper)
    timer.start(500)

    app.exec()

    print("等待 Copper 线程结束...")
    _rust.join_copper(2.0)
    print("完成。")


if __name__ == "__main__":
    main()
