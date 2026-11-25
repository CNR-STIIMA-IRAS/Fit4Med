# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""One bring-up at a time (runs 0111/0112): plc_manager.bringup_lock."""

import os
import subprocess
import sys
import textwrap
import time

import pytest

from plc_manager import bringup_lock

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture(autouse=True)
def isolated_lock(tmp_path, monkeypatch):
    if bringup_lock._bringup_processes():
        pytest.skip('a real bring-up is running on this PC')
    monkeypatch.setattr(bringup_lock, 'LOCK_FILE', str(tmp_path / 'ros' / 'bringup.lock'))
    monkeypatch.setattr(bringup_lock, '_lock_fd', None)
    yield
    if bringup_lock._lock_fd is not None:
        os.close(bringup_lock._lock_fd)


def _wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


def _start(argv, **kwargs):
    proc = subprocess.Popen(argv, stdout=subprocess.PIPE, text=True, **kwargs)
    return proc


def _stop(proc):
    proc.kill()
    proc.wait(timeout=5)


def test_free_pc_takes_the_lock():
    bringup_lock.check()
    bringup_lock.acquire()
    with open(bringup_lock.LOCK_FILE) as f:
        assert f.read().strip() == str(os.getpid())


def test_lock_held_by_another_launch_is_refused():
    holder = _start(
        [sys.executable, '-c', textwrap.dedent(f'''
            import sys, time
            sys.path.insert(0, {PACKAGE_ROOT!r})
            from plc_manager import bringup_lock
            bringup_lock.LOCK_FILE = {bringup_lock.LOCK_FILE!r}
            bringup_lock.acquire()
            print('locked', flush=True)
            time.sleep(30)
        ''')])
    try:
        assert holder.stdout.readline().strip() == 'locked'
        with pytest.raises(bringup_lock.BringupRunning, match=f'pid {holder.pid}'):
            bringup_lock.check()
        with pytest.raises(bringup_lock.BringupRunning, match=f'pid {holder.pid}'):
            bringup_lock.acquire()
        assert bringup_lock._lock_fd is None
    finally:
        _stop(holder)
    # Released by the kernel when the holder dies (SIGKILL here).
    bringup_lock.acquire()


def test_orphan_plc_manager_is_refused(tmp_path):
    # plc_manager_node is a Python entry point: argv[1] is the script.
    script = tmp_path / 'plc_manager_node'
    script.write_text('import time\ntime.sleep(30)\n')
    orphan = _start([sys.executable, str(script)])
    try:
        assert _wait_for(lambda: orphan.pid in bringup_lock._bringup_processes())
        with pytest.raises(bringup_lock.BringupRunning, match=f'pid {orphan.pid}'):
            bringup_lock.acquire()
        assert bringup_lock._lock_fd is None
    finally:
        _stop(orphan)
    bringup_lock.acquire()


def test_launch_without_lock_is_refused(tmp_path):
    # An older install: the launch does not take the lock.
    launch = _start(
        [sys.executable, '-c', 'import time; time.sleep(30)',
         'launch', 'tecnobody_workbench', 'run_sickPLC.launch.py', 'gui_ip:=192.168.1.2'])
    try:
        assert _wait_for(lambda: launch.pid in bringup_lock._bringup_processes())
        with pytest.raises(bringup_lock.BringupRunning):
            bringup_lock.check()
    finally:
        _stop(launch)


def test_shell_that_starts_the_launch_is_not_a_bringup():
    # fmrr_bringup.ps1 legacy mode: bash -c '... ros2 launch ... run_sickPLC.launch.py ...'
    shell = _start(
        ['bash', '-c', 'sleep 30; ros2 launch tecnobody_workbench '
         'run_sickPLC.launch.py gui_ip:=192.168.1.2; true'])
    try:
        time.sleep(0.2)
        assert shell.pid not in bringup_lock._bringup_processes()
        bringup_lock.acquire()
    finally:
        _stop(shell)


def test_module_exit_code():
    def run():
        return subprocess.run(
            [sys.executable, '-m', 'plc_manager.bringup_lock'], cwd=PACKAGE_ROOT,
            env={**os.environ, 'FIT4MED_BRINGUP_LOCK': bringup_lock.LOCK_FILE},
            capture_output=True, text=True)

    assert run().returncode == 0
    bringup_lock.acquire()
    result = run()
    assert result.returncode == 1
    assert 'another bring-up is running' in result.stderr
