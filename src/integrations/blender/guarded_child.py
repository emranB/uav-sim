"""Bind an owned offline renderer or screen encoder to its parent process lifetime."""

import ctypes
import os
import signal
import sys


def main() -> None:
    parent_pid = int(sys.argv[1])
    command = sys.argv[3:]
    if not command or sys.argv[2] != "--":
        raise ValueError("Supply the owned offline command after --")
    libc = ctypes.CDLL(None, use_errno=True)
    # Linux PR_SET_PDEATHSIG survives exec; no live UAV process is part of this group.
    if libc.prctl(1, signal.SIGKILL, 0, 0, 0) != 0:
        raise OSError(ctypes.get_errno(), "Cannot bind offline process lifetime to its owner")
    if os.getppid() != parent_pid:
        raise RuntimeError("Offline process owner ended before launch")
    os.execvpe(command[0], command, os.environ)


if __name__ == "__main__":
    main()
