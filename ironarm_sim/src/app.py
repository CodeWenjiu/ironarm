"""Application main window — hosts the embedded TUI terminal."""

from PySide6.QtWidgets import QMainWindow

from .terminal import TerminalWidget


class MainWindow(QMainWindow):
    def __init__(self, tui_binary: str) -> None:
        super().__init__()
        self.setWindowTitle("IronArm Simulator")
        self._terminal = TerminalWidget([tui_binary])
        self.setCentralWidget(self._terminal)
        self.resize(900, 550)
