"""IronArm Simulator — Python GUI with embedded copper TUI."""

import os
import sys

from PySide6.QtWidgets import QApplication
from src.main import MainWindow


def main() -> None:
    app = QApplication(sys.argv)

    # TUI binary is in the workspace target directory (two levels up).
    workspace_root = os.path.dirname(os.path.dirname(__file__))
    tui_binary = os.path.join(workspace_root, "target", "debug", "ironarm_tui")
    if not os.path.exists(tui_binary):
        raise FileNotFoundError(
            f"TUI binary not found: {tui_binary}\n"
            f'Run "cargo build -p ironarm_tui" first.'
        )

    window = MainWindow(tui_binary)
    window.show()
    app.exec()


if __name__ == "__main__":
    main()
