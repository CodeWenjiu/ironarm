"""Application main window — 3-D arm view."""

import atexit
import os
import sys

import ironarm_sim as _rust  # type: ignore[import-untyped]
from PySide6.QtCore import QTimer
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtWidgets import QApplication, QMainWindow

from .arm.view import Arm3DView


def _reset_terminal() -> None:
    if sys.stdout.isatty():
        os.system("stty sane 2>/dev/null")


atexit.register(_reset_terminal)


def _print_gpu_info() -> None:
    """Print GL vendor/renderer before copper TUI takes over the terminal."""
    app = QApplication.instance() or QApplication(sys.argv)
    widget = QOpenGLWidget()
    widget.resize(1, 1)
    widget.show()
    widget.makeCurrent()
    gl = widget.context().functions()
    vendor = gl.glGetString(0x1F00)  # GL_VENDOR
    renderer = gl.glGetString(0x1F01)  # GL_RENDERER
    if vendor:
        print(f"GPU: {vendor if isinstance(vendor, str) else vendor.decode()} — {renderer if isinstance(renderer, str) else renderer.decode()}")
    else:
        print("GPU: (no GL context)")
    widget.doneCurrent()
    widget.hide()
    widget.destroy()


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("IronArm Simulator — UR5e")
        self._view3d = Arm3DView()
        self.setCentralWidget(self._view3d)
        self.resize(900, 550)

    def closeEvent(self, event) -> None:
        print("Shutting down...")
        QApplication.quit()
        super().closeEvent(event)


def main() -> None:
    _print_gpu_info()
    _rust.start_copper()

    app = QApplication.instance()
    window = MainWindow()
    window.show()

    def _watch_copper() -> None:
        if not _rust.is_copper_alive():
            print("Copper exited, closing GUI.")
            app.quit()

    timer = QTimer()
    timer.timeout.connect(_watch_copper)
    timer.start(500)

    app.exec()

    print("Waiting for copper thread...")
    _rust.join_copper(2.0)
    print("Done.")


if __name__ == "__main__":
    main()
