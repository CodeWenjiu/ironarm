"""Application main window — 3-D arm view on the left, TUI terminal on the right."""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QMainWindow, QSplitter

from .arm.view import Arm3DView
from .terminal.widget import TerminalWidget


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
