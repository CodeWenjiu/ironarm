"""IronArm GUI with embedded copper TUI and simulation loop."""

import fcntl
import os
import pty
import signal
import struct
import sys
import termios
import warnings

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QFont, QPainter
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
)
from pyte import Screen, Stream
from pyte.screens import Char as PyteChar

# -- ANSI colour -> hex mapping -------------------------------------------------
_NORMAL: dict[str, str] = {
    "black": "#1a1a1a",
    "red": "#cc3333",
    "green": "#33cc33",
    "brown": "#cc8833",
    "blue": "#3366cc",
    "magenta": "#cc33cc",
    "cyan": "#33cccc",
    "white": "#cccccc",
}
_BRIGHT: dict[str, str] = {
    "black": "#555555",
    "red": "#ff5555",
    "green": "#55ff55",
    "brown": "#ffff55",
    "blue": "#5555ff",
    "magenta": "#ff55ff",
    "cyan": "#55ffff",
    "white": "#ffffff",
}


def _to_qcolor(name: str, bold: bool) -> QColor:
    """pyte colour string -> QColor."""
    if name == "default":
        return QColor(204, 204, 204) if not bold else QColor(255, 255, 255)
    # pyte uses 'rrggbb' WITHOUT # for 256-colour and truecolour
    if len(name) == 6 and all(c in "0123456789abcdef" for c in name.lower()):
        return QColor("#" + name)
    if name.startswith("#"):
        return QColor(name)
    if bold and name in _BRIGHT:
        return QColor(_BRIGHT[name])
    if name in _NORMAL:
        return QColor(_NORMAL[name])
    return QColor(204, 204, 204)


# -- PTY spawn helper -----------------------------------------------------------


def _spawn_in_pty(argv: list[str]) -> tuple[int, int]:
    """Fork a child with a proper controlling PTY.

    Returns (child_pid, master_fd).
    The child will exec *argv with stdin/stdout/stderr connected to the PTY slave,
    and the PTY slave properly set as its controlling terminal.
    """
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)

    # PySide6 threads hold locks at fork time, but the child only calls
    # async-signal-safe syscalls (setsid, open, dup2, execve) so the
    # theoretical deadlock cannot materialise.
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", ".*fork.*thread.*")
        pid = os.fork()
    if pid == 0:
        # --- child ---
        os.close(master_fd)
        os.setsid()  # new session; drops any existing controlling tty

        # Open the slave by name to acquire it as controlling terminal.
        fd = os.open(slave_name, os.O_RDWR)
        os.close(slave_fd)

        os.dup2(fd, 0)
        os.dup2(fd, 1)
        os.dup2(fd, 2)
        if fd > 2:
            os.close(fd)

        env = os.environ.copy()
        env["TERM"] = "xterm-256color"
        os.execve(argv[0], argv, env)
        os._exit(127)  # execve never returns on success

    # --- parent ---
    os.close(slave_fd)
    return pid, master_fd


# -- Widgets --------------------------------------------------------------------


class TerminalWidget(QWidget):
    """Embedded terminal that runs copper TUI via PTY."""

    _CHAR_W = 8
    _CHAR_H = 16

    _KEY_MAP: dict[int, str] = {
        Qt.Key.Key_Up: "\x1b[A",
        Qt.Key.Key_Down: "\x1b[B",
        Qt.Key.Key_Right: "\x1b[C",
        Qt.Key.Key_Left: "\x1b[D",
        Qt.Key.Key_Home: "\x1b[H",
        Qt.Key.Key_End: "\x1b[F",
        Qt.Key.Key_PageUp: "\x1b[5~",
        Qt.Key.Key_PageDown: "\x1b[6~",
        Qt.Key.Key_Insert: "\x1b[2~",
        Qt.Key.Key_Delete: "\x1b[3~",
        Qt.Key.Key_Backspace: "\x7f",
        Qt.Key.Key_Escape: "\x1b",
        Qt.Key.Key_Return: "\r",
        Qt.Key.Key_Enter: "\r",
        Qt.Key.Key_Tab: "\t",
        Qt.Key.Key_F1: "\x1bOP",
        Qt.Key.Key_F2: "\x1bOQ",
        Qt.Key.Key_F3: "\x1bOR",
        Qt.Key.Key_F4: "\x1bOS",
        Qt.Key.Key_F5: "\x1b[15~",
        Qt.Key.Key_F6: "\x1b[17~",
        Qt.Key.Key_F7: "\x1b[18~",
        Qt.Key.Key_F8: "\x1b[19~",
        Qt.Key.Key_F9: "\x1b[20~",
        Qt.Key.Key_F10: "\x1b[21~",
        Qt.Key.Key_F11: "\x1b[23~",
        Qt.Key.Key_F12: "\x1b[24~",
    }

    def __init__(self, cmd: list[str], parent=None):
        super().__init__(parent)
        cols, rows = 80, 24
        self._screen = Screen(cols, rows)
        self.stream = Stream(self._screen)
        self.cmd = cmd
        self._pid: int | None = None
        self._master_fd: int | None = None
        self._font = QFont("Noto Sans Mono", 10)
        self._font.setStyleHint(QFont.StyleHint.Monospace)
        # Cache cell geometry after font metrics are available
        self._cw: float = float(self._CHAR_W)
        self._ch: int = self._CHAR_H
        self._cols = cols
        self._rows = rows
        self.setMinimumSize(400, 300)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        QTimer.singleShot(100, self._start)

    def _start(self):
        pid, master_fd = _spawn_in_pty(self.cmd)
        self._pid = pid
        self._master_fd = master_fd
        self._set_pty_size(master_fd)
        self._reader = QTimer(self)
        self._reader.timeout.connect(self._read)
        self._reader.start(16)

    def _set_pty_size(self, fd: int) -> None:
        w = self.width()
        h = self.height()
        self._cols = max(20, w // self._CHAR_W)
        self._rows = max(5, h // self._CHAR_H)
        try:
            winsize = struct.pack("HHHH", self._rows, self._cols, w, h)
            fcntl.ioctl(fd, termios.TIOCSWINSZ, winsize)
        except OSError:
            pass
        if self._cols != self._screen.columns or self._rows != self._screen.lines:
            self._screen.resize(lines=self._rows, columns=self._cols)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._master_fd is not None:
            self._set_pty_size(self._master_fd)
            if self._pid is not None:
                try:
                    os.kill(self._pid, signal.SIGWINCH)
                except OSError:
                    pass

    def _read(self):
        if self._master_fd is None:
            return
        try:
            wpid, status = os.waitpid(self._pid or 0, os.WNOHANG)
            if wpid != 0:
                code = os.WEXITSTATUS(status) if os.WIFEXITED(status) else -1
                print(f"TUI exited with code {code}", file=sys.stderr)
                self._reader.stop()
                # Close the whole application when the TUI quits
                from PySide6.QtWidgets import QApplication
                QApplication.instance().quit()
                return
        except ChildProcessError:
            pass

        try:
            data = os.read(self._master_fd, 4096)
            if data:
                decoded = data.decode("utf-8", errors="replace")
                self.stream.feed(decoded)
                self.update()
        except OSError:
            pass

    def _write_to_child(self, data: bytes) -> None:
        if self._master_fd is not None:
            try:
                os.write(self._master_fd, data)
            except OSError:
                pass

    # -- keyboard input ---------------------------------------------------------

    def keyPressEvent(self, event):
        if self._master_fd is None:
            super().keyPressEvent(event)
            return

        key = event.key()
        modifiers = event.modifiers()
        ctrl = bool(modifiers & Qt.KeyboardModifier.ControlModifier)
        shift = bool(modifiers & Qt.KeyboardModifier.ShiftModifier)
        alt = bool(modifiers & Qt.KeyboardModifier.AltModifier)

        if ctrl and not alt and Qt.Key.Key_A <= key <= Qt.Key.Key_Z:
            self._write_to_child(chr(key - Qt.Key.Key_A + 1).encode())
            super().keyPressEvent(event)
            return

        if alt and not ctrl:
            text = event.text()
            if text:
                self._write_to_child(b"\x1b" + text.encode())
                super().keyPressEvent(event)
                return

        seq = self._KEY_MAP.get(key)
        if seq is not None:
            if shift and Qt.Key.Key_F1 <= key <= Qt.Key.Key_F4:
                shifted: dict[int, str] = {
                    Qt.Key.Key_F1: "\x1b[1;2P",
                    Qt.Key.Key_F2: "\x1b[1;2Q",
                    Qt.Key.Key_F3: "\x1b[1;2R",
                    Qt.Key.Key_F4: "\x1b[1;2S",
                }
                seq = shifted.get(key, seq)
            self._write_to_child(seq.encode())
            super().keyPressEvent(event)
            return

        text = event.text()
        if text:
            self._write_to_child(text.encode())

        super().keyPressEvent(event)

    # -- rendering (colour-aware) -----------------------------------------------

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(0x1A, 0x1A, 0x1A))
        painter.setFont(self._font)

        # Compute exact monospace cell size (float for subpixel accuracy)
        fm = painter.fontMetrics()
        self._cw = fm.horizontalAdvance("0")
        self._ch = float(fm.lineSpacing())

        for row, line in self._screen.buffer.items():
            y = row * self._ch
            for col, ch in line.items():
                x = col * self._cw

                bg = self._bg_color(ch)
                if bg is not None:
                    painter.fillRect(x, y, self._cw, self._ch, bg)

                fg = _to_qcolor(ch.fg, ch.bold)
                painter.setPen(fg)

                text = ch.data
                painter.drawText(float(x), float(y + fm.ascent()), text)

    def _bg_color(self, ch: PyteChar) -> QColor | None:
        """Return background QColor for a cell, or None for default."""
        if ch.bg == "default":
            return None
        if ch.reverse:
            return _to_qcolor(ch.fg, ch.bold)
        # pyte uses 'rrggbb' hex without # prefix
        if len(ch.bg) == 6 and all(c in "0123456789abcdef" for c in ch.bg.lower()):
            return QColor("#" + ch.bg)
        if ch.bg.startswith("#"):
            return QColor(ch.bg)
        if ch.bg in _NORMAL:
            return QColor(_NORMAL[ch.bg])
        if ch.bg in _BRIGHT:
            return QColor(_BRIGHT[ch.bg])
        return None

    def closeEvent(self, event):
        if self._pid is not None:
            try:
                os.kill(self._pid, signal.SIGTERM)
                os.waitpid(self._pid, 0)
            except OSError:
                pass
        super().closeEvent(event)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IronArm Simulator")

        tui_binary = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            "target/debug/ironarm_tui",
        )
        if not os.path.exists(tui_binary):
            raise FileNotFoundError(
                f"TUI binary not found at {tui_binary}. "
                f'Run "cargo build -p ironarm_tui" first.'
            )

        self._terminal = TerminalWidget([tui_binary])
        self.setCentralWidget(self._terminal)
        self.resize(900, 550)


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec()


if __name__ == "__main__":
    main()
