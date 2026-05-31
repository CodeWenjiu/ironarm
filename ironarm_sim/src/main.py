"""Application main window — 3-D arm view."""

import sys

import ironarm_sim as _rust  # type: ignore[import-untyped]
from PySide6.QtWidgets import QApplication, QMainWindow

from .arm.view import Arm3DView


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("IronArm Simulator")
        self._view3d = Arm3DView()
        self.setCentralWidget(self._view3d)
        self.resize(900, 550)
        _rust.register_callback(self._view3d._on_angles)


def main() -> None:
    _rust.start_copper()
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec()


if __name__ == "__main__":
    main()
