# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
"""One PLC bring-up (run_sickPLC.launch.py) at a time on the robot PC.

A second bring-up cannot work (EtherCAT master 0 and UDP port 5006 are
taken) and, failing, used to stop the first one: in runs 0111/0112 a launch
from a terminal and the systemd unit started 11 s apart, the unit's
plc_manager died on the busy port and its shutdown killed the plc_manager of
the other launch, which opened the safety chain.

run_sickPLC.launch.py takes the lock with acquire() before starting any
process and keeps it until it exits (also SIGKILL: the kernel releases it).
The scripts that start it run `python3 -m plc_manager.bringup_lock` first,
before touching the logs of the bring-up that is running.

The lock alone misses a launch that does not take it (an older install) and
nodes left behind by a killed launch, so the processes are checked as well.
"""

import fcntl
import os
import sys

LOCK_FILE = os.environ.get(
    'FIT4MED_BRINGUP_LOCK',
    os.path.join(os.path.expanduser('~'), '.ros', 'fit4med_bringup.lock'))

LAUNCH_FILE = 'run_sickPLC.launch.py'
PLC_MANAGER = 'plc_manager_node'
CLEANUP_HINT = ('Stop it first (fmrr_cleanup.ps1 from the GUI PC, '
                "or Ctrl-C in the terminal that started it).")

# Kept open while this process runs: closing it releases the lock.
_lock_fd: int | None = None


class BringupRunning(RuntimeError):
    """Another bring-up is running on this PC."""


def _bringup_processes() -> dict[int, str]:
    """Return {pid: command line} of this user's other bring-up processes.

    These are a run_sickPLC launch or a plc_manager_node, matched by
    the exact name of an argument so that the shell that started this launch
    (`bash -c '... ros2 launch ... run_sickPLC.launch.py ...'`) is not one.
    """
    own_pid, own_uid = os.getpid(), os.getuid()
    found = {}
    for entry in os.listdir('/proc'):
        if not entry.isdigit() or int(entry) == own_pid:
            continue
        try:
            if os.stat(f'/proc/{entry}').st_uid != own_uid:
                continue
            with open(f'/proc/{entry}/cmdline', 'rb') as f:
                argv = [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
        except OSError:
            continue
        names = [os.path.basename(a) for a in argv]
        # plc_manager_node: argv[0] as executable, argv[1] as "python3 <script>".
        if LAUNCH_FILE in names or PLC_MANAGER in names[:2]:
            found[int(entry)] = ' '.join(argv)
    return found


def _lock_holder() -> str:
    try:
        with open(LOCK_FILE) as f:
            return f.read().strip() or '?'
    except OSError:
        return '?'


def _describe(processes: dict[int, str]) -> str:
    return '; '.join(f'pid {pid}: {cmd[:120]}' for pid, cmd in sorted(processes.items()))


def _try_lock(fd: int, mode: int) -> None:
    try:
        fcntl.flock(fd, mode | fcntl.LOCK_NB)
    except BlockingIOError:
        raise BringupRunning(
            f'another bring-up is running (pid {_lock_holder()}, '
            f'lock {LOCK_FILE}). {CLEANUP_HINT}') from None


def _check_processes() -> None:
    others = _bringup_processes()
    if others:
        raise BringupRunning(
            f'bring-up processes still running: {_describe(others)}. {CLEANUP_HINT}')


def check() -> None:
    """Raise BringupRunning if another bring-up is running; take no lock."""
    if os.path.isdir(os.path.dirname(LOCK_FILE)):
        fd = os.open(LOCK_FILE, os.O_RDWR | os.O_CREAT, 0o644)
        try:
            _try_lock(fd, fcntl.LOCK_SH)
        finally:
            os.close(fd)
    _check_processes()


def acquire() -> None:
    """Take the bring-up lock for this process, or raise BringupRunning."""
    global _lock_fd
    if _lock_fd is not None:
        return
    os.makedirs(os.path.dirname(LOCK_FILE), exist_ok=True)
    fd = os.open(LOCK_FILE, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        _try_lock(fd, fcntl.LOCK_EX)
        _check_processes()
        os.ftruncate(fd, 0)
        os.write(fd, f'{os.getpid()}\n'.encode())
    except BaseException:
        os.close(fd)
        raise
    # Not inherited by the launched processes (os.open is non-inheritable):
    # the lock goes with this process only.
    _lock_fd = fd


def main() -> int:
    """`python3 -m plc_manager.bringup_lock`: exit 1 if a bring-up is running."""
    try:
        check()
    except BringupRunning as exc:
        print(f'ERROR: {exc}', file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
