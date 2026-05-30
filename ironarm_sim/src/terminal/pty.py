"""Fork a child process connected to a proper controlling PTY."""

import os
import pty
import warnings


def spawn_in_pty(argv: list[str]) -> tuple[int, int]:
    """Fork and exec *argv with the PTY slave as controlling terminal.

    Returns ``(child_pid, master_fd)``.

    The child re-opens the slave device by name so that it becomes the
    session's controlling terminal — this is required by terminal-UI
    programs (e.g. ratatui) that call ``tcgetpgrp`` to verify they own
    the foreground process group.
    """
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)

    # The parent is multi-threaded (PySide6), but the child only executes
    # async-signal-safe syscalls before execve, so a deadlock cannot occur.
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", ".*fork.*thread.*")
        pid = os.fork()

    if pid == 0:  # ---- child ----
        os.close(master_fd)
        os.setsid()

        fd = os.open(slave_name, os.O_RDWR)  # acquire controlling tty
        os.close(slave_fd)

        os.dup2(fd, 0)
        os.dup2(fd, 1)
        os.dup2(fd, 2)
        if fd > 2:
            os.close(fd)

        env = os.environ.copy()
        env["TERM"] = "xterm-256color"
        os.execve(argv[0], argv, env)
        os._exit(127)  # execve never returns

    # ---- parent ----
    os.close(slave_fd)
    return pid, master_fd
