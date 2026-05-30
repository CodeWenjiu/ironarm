"""Embedded terminal widget — runs a process in a PTY and renders via pyte."""

import fcntl
import os
import signal
import struct
import sys
import termios

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer
from PySide6.QtGui import QColor, QFont, QPainter
from PySide6.QtWidgets import QApplication, QWidget
from pyte import Screen, Stream
from pyte.screens import Char as PyteChar

from .colors import _BRIGHT, _NORMAL, to_qcolor
from .pty import spawn_in_pty

# Cell size used before the first paintEvent supplies real font metrics.
_CHAR_W = 8
_CHAR_H = 16


class TerminalWidget(QWidget):
    """A ``QWidget`` that embeds a full terminal emulator.

    The child process is spawned with a real controlling PTY so that TUI
    programs (ratatui, ncurses, …) see an interactive terminal rather than
    a dumb pipe.  Rendering is done cell-by-cell to preserve ANSI colours.
    """

    # Qt key → ANSI escape sequence
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

    def __init__(self, cmd: list[str], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._cmd = cmd
        self._pid: int | None = None
        self._master_fd: int | None = None

        # pyte screen buffer
        self._cols = 80
        self._rows = 24
        self._screen = Screen(self._cols, self._rows)
        self._stream = Stream(self._screen)

        # Cell geometry (refined on first paintEvent)
        self._cw: float = float(_CHAR_W)
        self._ch: float = float(_CHAR_H)

        # Font
        self._font = QFont("Noto Sans Mono", 10)
        self._font.setStyleHint(QFont.StyleHint.Monospace)

        self.setMinimumSize(400, 300)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        # Defer PTY setup to avoid blocking the event loop
        QTimer.singleShot(0, self._start)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def _start(self) -> None:
        self._pid, self._master_fd = spawn_in_pty(self._cmd)
        self._set_pty_size(self._master_fd)

        self._reader = QTimer(self)
        self._reader.timeout.connect(self._read)
        self._reader.start(16)  # ≈ 60 fps read interval

    def closeEvent(self, event) -> None:
        if self._pid is not None:
            try:
                os.kill(self._pid, signal.SIGTERM)
                os.waitpid(self._pid, 0)
            except OSError:
                pass
        super().closeEvent(event)

    # ------------------------------------------------------------------
    # PTY read / write / resize
    # ------------------------------------------------------------------

    def _read(self) -> None:
        if self._master_fd is None:
            return

        # Detect child exit
        try:
            wpid, status = os.waitpid(self._pid or 0, os.WNOHANG)
            if wpid != 0:
                code = os.WEXITSTATUS(status) if os.WIFEXITED(status) else -1
                print(f"TUI exited with code {code}", file=sys.stderr)
                self._reader.stop()
                app = QApplication.instance()
                assert app is not None
                app.quit()
                return
        except ChildProcessError:
            pass

        try:
            data = os.read(self._master_fd, 4096)
            if data:
                self._stream.feed(data.decode("utf-8", errors="replace"))
                self.update()
        except OSError:
            pass

    def _write_to_child(self, data: bytes) -> None:
        if self._master_fd is not None:
            try:
                os.write(self._master_fd, data)
            except OSError:
                pass

    def _set_pty_size(self, fd: int) -> None:
        w, h = self.width(), self.height()
        self._cols = max(20, w // _CHAR_W)
        self._rows = max(5, h // _CHAR_H)
        try:
            winsize = struct.pack("HHHH", self._rows, self._cols, w, h)
            fcntl.ioctl(fd, termios.TIOCSWINSZ, winsize)
        except OSError:
            pass
        if self._cols != self._screen.columns or self._rows != self._screen.lines:
            self._screen.resize(lines=self._rows, columns=self._cols)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        if self._master_fd is not None:
            self._set_pty_size(self._master_fd)
            if self._pid is not None:
                try:
                    os.kill(self._pid, signal.SIGWINCH)
                except OSError:
                    pass

    # ------------------------------------------------------------------
    # Keyboard input
    # ------------------------------------------------------------------

    def keyPressEvent(self, event) -> None:
        if self._master_fd is None:
            super().keyPressEvent(event)
            return

        key = event.key()
        mods = event.modifiers()
        ctrl = bool(mods & Qt.KeyboardModifier.ControlModifier)
        shift = bool(mods & Qt.KeyboardModifier.ShiftModifier)
        alt = bool(mods & Qt.KeyboardModifier.AltModifier)

        # Ctrl+Letter → control character
        if ctrl and not alt and Qt.Key.Key_A <= key <= Qt.Key.Key_Z:
            self._write_to_child(chr(key - Qt.Key.Key_A + 1).encode())
            super().keyPressEvent(event)
            return

        # Alt+Letter → ESC prefix
        if alt and not ctrl:
            text = event.text()
            if text:
                self._write_to_child(b"\x1b" + text.encode())
                super().keyPressEvent(event)
                return

        # Special keys (arrows, F-keys, …)
        seq = self._KEY_MAP.get(key)
        if seq is not None:
            if shift and Qt.Key.Key_F1 <= key <= Qt.Key.Key_F4:
                seq = {
                    Qt.Key.Key_F1: "\x1b[1;2P",
                    Qt.Key.Key_F2: "\x1b[1;2Q",
                    Qt.Key.Key_F3: "\x1b[1;2R",
                    Qt.Key.Key_F4: "\x1b[1;2S",
                }.get(key, seq)
            self._write_to_child(seq.encode())
            super().keyPressEvent(event)
            return

        # Printable characters
        text = event.text()
        if text:
            self._write_to_child(text.encode())

        super().keyPressEvent(event)

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(0x1A, 0x1A, 0x1A))
        painter.setFont(self._font)

        fm = painter.fontMetrics()
        self._cw = fm.horizontalAdvance("0")
        self._ch = float(fm.lineSpacing())

        for row, line in self._screen.buffer.items():
            y = row * self._ch
            for col, ch in line.items():
                x = col * self._cw

                bg = self._bg_color(ch)
                if bg is not None:
                    painter.fillRect(QRectF(x, y, self._cw, self._ch), bg)  # pyright: ignore[reportArgumentType]

                painter.setPen(to_qcolor(ch.fg, ch.bold))
                painter.drawText(QPointF(x, y + fm.ascent()), ch.data)  # pyright: ignore[reportArgumentType]

    @staticmethod
    def _bg_color(ch: PyteChar) -> QColor | None:
        """Return the background ``QColor`` for a cell, or ``None`` for default."""
        if ch.bg == "default":
            return None
        if ch.reverse:
            return to_qcolor(ch.fg, ch.bold)
        # 6-digit hex without '#' (pyte 256-colour / truecolour representation)
        if len(ch.bg) == 6 and all(c in "0123456789abcdef" for c in ch.bg.lower()):
            return QColor("#" + ch.bg)
        if ch.bg.startswith("#"):
            return QColor(ch.bg)
        if ch.bg in _NORMAL:
            return QColor(_NORMAL[ch.bg])
        if ch.bg in _BRIGHT:
            return QColor(_BRIGHT[ch.bg])
        return None
