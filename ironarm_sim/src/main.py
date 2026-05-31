"""Application main window — 3-D arm view."""

import atexit
import os
import sys

import ironarm_sim as _rust  # type: ignore[import-untyped]
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication, QMainWindow

from .arm.view import Arm3DView


def _reset_terminal() -> None:
    """Restore terminal after copper TUI raw mode."""
    if sys.stdout.isatty():
        os.system("stty sane 2>/dev/null")


atexit.register(_reset_terminal)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("IronArm Simulator")
        self._view3d = Arm3DView()
        self.setCentralWidget(self._view3d)
        self.resize(900, 550)

    def closeEvent(self, event) -> None:
        print("Shutting down...")
        QApplication.quit()
        super().closeEvent(event)


def main() -> None:
    _rust.start_copper()
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    def _watch_copper() -> None:
        if not _rust.is_copper_alive():
            print("Copper TUI exited, closing GUI.")
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
