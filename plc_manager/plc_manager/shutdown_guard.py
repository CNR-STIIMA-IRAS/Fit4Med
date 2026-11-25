# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
"""Run a command and stop it only after other processes have exited.

    shutdown_guard [--after NAME]... [--max-wait SEC] -- CMD [ARGS...]

Used as launch prefix of the PLC ros2_control_node (run_sickPLC.launch.py).
On shutdown plc_manager raises the SW ESTOP through that node and waits for
the PLC to confirm it. If the node gets SIGINT together with plc_manager (as
with launch's own shutdown, a Ctrl-C in the terminal, or systemd signalling
the whole unit), the EtherCAT master stops first, the command never reaches
the PLC and plc_manager's shutdown watchdog expires (run 0092).

The first SIGINT is therefore not forwarded at once: the guard waits until
the processes named by --after (e.g. plc_manager_node) have exited, at most
--max-wait seconds, then sends SIGINT to the command. SIGTERM (launch's
escalation) is forwarded immediately. The command runs in its own session,
so a terminal Ctrl-C does not reach it directly, and it gets SIGINT if the
guard itself dies (e.g. SIGKILL).
"""

import argparse
import ctypes
import os
import signal
import subprocess
import sys
import time

PR_SET_PDEATHSIG = 1
POLL_SEC = 0.1


def _log(msg: str) -> None:
    print(f'[shutdown_guard] {msg}', file=sys.stderr, flush=True)


def _parent_pid(pid: str) -> int:
    with open(f'/proc/{pid}/stat') as f:
        # ppid is the second field after "(comm)"; comm may contain spaces.
        return int(f.read().rsplit(')', 1)[1].split()[1])


def _pids_named(names: set[str]) -> list[int]:
    """Return the pids of the sibling processes running a program in names.

    Siblings: started by the same launch as this guard. A plc_manager_node of
    another bring-up is not waited for (run 0112 waited for the one of 0111).
    """
    own_pid, own_ppid = os.getpid(), os.getppid()
    pids = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit() or int(entry) == own_pid:
            continue
        try:
            if _parent_pid(entry) != own_ppid:
                continue
            with open(f'/proc/{entry}/cmdline', 'rb') as f:
                argv = f.read().split(b'\0')
        except (OSError, ValueError, IndexError):
            continue
        # argv[0] for an executable, argv[1] for "python3 <script>".
        if any(os.path.basename(a.decode(errors='replace')) in names for a in argv[:2]):
            pids.append(int(entry))
    return pids


def _alive(pid: int) -> bool:
    try:
        with open(f'/proc/{pid}/stat') as f:
            # state is the field after "(comm)"; comm may contain spaces.
            return f.read().rsplit(')', 1)[1].split()[0] != 'Z'
    except (OSError, IndexError):
        return False


def _set_parent_death_signal() -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    libc.prctl(PR_SET_PDEATHSIG, signal.SIGINT)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--after', action='append', default=[], metavar='NAME',
                        help='process to wait for before stopping CMD (repeatable)')
    parser.add_argument('--max-wait', type=float, default=15.0, metavar='SEC',
                        help='longest wait for those processes (default 15)')
    parser.add_argument('cmd', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    cmd = args.cmd[1:] if args.cmd[:1] == ['--'] else args.cmd
    if not cmd:
        parser.error('missing command')

    received: list[int] = []
    signal.signal(signal.SIGINT, lambda signum, _frame: received.append(signum))
    signal.signal(signal.SIGTERM, lambda signum, _frame: received.append(signum))

    child = subprocess.Popen(cmd, start_new_session=True,
                             preexec_fn=_set_parent_death_signal)

    stop_requested = False
    sigint_sent = False
    sigterm_sent = False
    waiting_for: list[int] = []
    deadline = 0.0
    while True:
        try:
            returncode = child.wait(timeout=POLL_SEC)
            break
        except subprocess.TimeoutExpired:
            pass

        if received and not stop_requested:
            stop_requested = True
            deadline = time.monotonic() + args.max_wait
            waiting_for = _pids_named(set(args.after))
            if waiting_for:
                _log(f'stop requested: waiting for {", ".join(args.after)} '
                     f'(pids {waiting_for}, max {args.max_wait:g} s) before stopping {cmd[0]}')

        if signal.SIGTERM in received and not sigterm_sent:
            _log(f'SIGTERM: forwarding to {cmd[0]} now')
            child.send_signal(signal.SIGTERM)
            sigterm_sent = sigint_sent = True

        if stop_requested and not sigint_sent:
            waiting_for = [pid for pid in waiting_for if _alive(pid)]
            timed_out = time.monotonic() >= deadline
            if not waiting_for or timed_out:
                if timed_out:
                    _log(f'still running after {args.max_wait:g} s: {waiting_for}')
                _log(f'forwarding SIGINT to {cmd[0]}')
                child.send_signal(signal.SIGINT)
                sigint_sent = True

    return 128 - returncode if returncode < 0 else returncode


if __name__ == '__main__':
    sys.exit(main())
