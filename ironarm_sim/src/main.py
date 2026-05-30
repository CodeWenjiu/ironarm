"""Application main window — 3-D arm view on the left, TUI terminal on the right."""

import os
import sys

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication, QMainWindow, QSplitter

from .arm.view import Arm3DView
from .terminal.widget import TerminalWidget

# TUI binary lives in the workspace root's target directory.
_WORKSPACE_ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
_TUI_BINARY = os.path.join(_WORKSPACE_ROOT, "target", "debug", "ironarm_tui")


class MainWindow(QMainWindow):
    def __init__(self, tui_binary: str) -> None:
        super().__init__()
        self.setWindowTitle("IronArm Simulator")

        self._view3d = Arm3DView()
        self._terminal = TerminalWidget([tui_binary])

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self._view3d)
        splitter.addWidget(self._terminal)
        splitter.setSizes([500, 500])
        self.setCentralWidget(splitter)
        self.resize(1200, 600)


def main() -> None:
    if not os.path.exists(_TUI_BINARY):
        raise FileNotFoundError(
            f"TUI binary not found: {_TUI_BINARY}\n"
            f'Run "cargo build -p ironarm_tui" first.'
        )
    app = QApplication(sys.argv)
    window = MainWindow(_TUI_BINARY)
    window.show()
    app.exec()


if __name__ == "__main__":
    main()
