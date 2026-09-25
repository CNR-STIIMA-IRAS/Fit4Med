# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""Per-session GUI log files (in C:\\temp on Windows).

    pyqt_hang.log    faulthandler: all thread stacks every 10 s and on a fatal error
    gui_errors.log   uncaught exceptions rendered by rich (with locals, no colours)
    gui_console.log  copy of everything printed to stdout/stderr, timestamped

At every GUI start the log of the previous run is moved to
<name>_<YYYYmmdd_HHMMSS>.log (timestamp = last write of that run, i.e. about
when it crashed/was closed) and only the newest MAX_BACKUPS backups are kept.
"""

import faulthandler
import glob
import os
import re
import shutil
import sys
import tempfile
import threading
import traceback
from datetime import datetime

from rich.console import Console
from rich.traceback import Traceback, install as install_rich_traceback

LOG_DIR = r"C:\temp" if sys.platform == "win32" else tempfile.gettempdir()
HANG_LOG_NAME = "pyqt_hang"
ERROR_LOG_NAME = "gui_errors"
CONSOLE_LOG_NAME = "gui_console"
MAX_BACKUPS = 20

# faulthandler keeps only the fd and the hooks the file objects: keep them alive.
_open_logs = {}

_ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")


def _backup_previous(path: str, stem: str) -> None:
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return
    stamp = datetime.fromtimestamp(os.path.getmtime(path)).strftime("%Y%m%d_%H%M%S")
    backup = f"{stem}_{stamp}.log"
    n = 1
    while os.path.exists(backup):
        backup = f"{stem}_{stamp}_{n}.log"
        n += 1
    try:
        os.replace(path, backup)
    except OSError:
        # Still held open by another (stuck) GUI instance: keep a copy and
        # go on appending to the same file.
        shutil.copy2(path, backup)


def _prune_backups(stem: str, keep: int) -> None:
    # The timestamp format sorts lexically in chronological order.
    backups = sorted(glob.glob(f"{stem}_*.log"))
    for old in backups[:-keep] if keep > 0 else backups:
        try:
            os.remove(old)
        except OSError as exc:
            print(f"[session_log] cannot remove old backup {old}: {exc}")


def _open_rotated(name: str):
    """Rotate <name>.log, open a fresh one and write the session header."""
    if name in _open_logs:
        return _open_logs[name]
    os.makedirs(LOG_DIR, exist_ok=True)
    stem = os.path.join(LOG_DIR, name)
    path = stem + ".log"
    try:
        _backup_previous(path, stem)
        _prune_backups(stem, MAX_BACKUPS)
    except OSError as exc:
        print(f"[session_log] rotation of {path} failed: {exc}")

    log_file = open(path, "a", buffering=1, encoding="utf-8", errors="replace")
    log_file.write(f"=== GUI session start {datetime.now():%Y-%m-%d %H:%M:%S} "
                   f"pid={os.getpid()} argv={sys.argv} ===\n")
    _open_logs[name] = log_file
    return log_file


def setup_hang_log(dump_period_s: float = 10.0) -> None:
    """faulthandler on pyqt_hang.log: periodic stack dumps + fatal errors."""
    log_file = _open_rotated(HANG_LOG_NAME)
    faulthandler.enable(file=log_file)
    faulthandler.dump_traceback_later(dump_period_s, repeat=True, file=log_file)


class _Tee:
    """Forward writes to the original stream and, timestamped, to a log file."""

    def __init__(self, stream, log_file):
        self._stream = stream
        self._log_file = log_file
        self._lock = threading.Lock()
        self._at_line_start = True

    def write(self, text):
        if self._stream is not None:
            try:
                self._stream.write(text)
            except Exception:
                pass
        with self._lock:
            try:
                clean = _ANSI_RE.sub("", text)
                out = []
                for line in clean.splitlines(keepends=True):
                    if self._at_line_start:
                        out.append(datetime.now().strftime("%H:%M:%S.%f")[:-3] + " ")
                    out.append(line)
                    self._at_line_start = line.endswith("\n")
                self._log_file.write("".join(out))
            except Exception:
                pass
        return len(text)

    def flush(self):
        for target in (self._stream, self._log_file):
            try:
                if target is not None:
                    target.flush()
            except Exception:
                pass

    def isatty(self):
        # Lets rich keep colours on the terminal; they are stripped for the file.
        return self._stream is not None and self._stream.isatty()

    def __getattr__(self, name):
        # encoding, fileno, errors, ... come from the original stream.
        return getattr(self._stream, name)


def setup_console_log() -> None:
    """Copy stdout/stderr to gui_console.log (what the GUI terminal shows)."""
    log_file = _open_rotated(CONSOLE_LOG_NAME)
    if not isinstance(sys.stdout, _Tee):
        sys.stdout = _Tee(sys.stdout, log_file)
    if not isinstance(sys.stderr, _Tee):
        sys.stderr = _Tee(sys.stderr, log_file)


def setup_error_log() -> None:
    """rich tracebacks on the terminal (as before) and in gui_errors.log.

    PyQt5 aborts the process after an exception escapes a slot, but it calls
    sys.excepthook first, so the traceback reaches the file before the abort.
    """
    install_rich_traceback(show_locals=True)
    terminal_hook = sys.excepthook
    log_file = _open_rotated(ERROR_LOG_NAME)
    file_console = Console(file=log_file, width=160, color_system=None,
                           force_terminal=False, soft_wrap=False)
    lock = threading.Lock()

    def log_exception(exc_type, exc_value, exc_tb, thread_name):
        with lock:
            try:
                file_console.rule(f"{datetime.now():%Y-%m-%d %H:%M:%S} "
                                  f"uncaught {exc_type.__name__} in thread {thread_name}")
                file_console.print(Traceback.from_exception(
                    exc_type, exc_value, exc_tb, width=160, show_locals=True,
                    locals_max_length=10, locals_max_string=200))
            except Exception:
                # rich itself failed (e.g. a broken __repr__ among the locals).
                traceback.print_exception(exc_type, exc_value, exc_tb, file=log_file)
            log_file.flush()

    def excepthook(exc_type, exc_value, exc_tb):
        log_exception(exc_type, exc_value, exc_tb, threading.current_thread().name)
        terminal_hook(exc_type, exc_value, exc_tb)

    def thread_excepthook(args):
        if args.exc_type is SystemExit:
            return
        thread_name = args.thread.name if args.thread is not None else "?"
        log_exception(args.exc_type, args.exc_value, args.exc_traceback, thread_name)
        terminal_hook(args.exc_type, args.exc_value, args.exc_traceback)

    sys.excepthook = excepthook
    threading.excepthook = thread_excepthook


def setup_session_logs() -> None:
    setup_console_log()  # first, so the other setup messages land in it too
    setup_hang_log()
    setup_error_log()
    print(f"[session_log] logs in {LOG_DIR}: {HANG_LOG_NAME}.log, "
          f"{ERROR_LOG_NAME}.log, {CONSOLE_LOG_NAME}.log")
