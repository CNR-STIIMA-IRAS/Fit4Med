#!/usr/bin/env python3
import os
import signal
import subprocess
import time

from rclpy.impl.rcutils_logger import RcutilsLogger as Logger

class bcolors:
    """ANSI color codes for terminal output.
    
    Provides colored logging for state transitions and critical events.
    """
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    MAGENTA = '\033[1;35m'


def find_matching_pids(pattern: str) -> list[int]:
    try:
        output = subprocess.check_output(
            ["pgrep", "-f", pattern],
            text=True
        )
    except subprocess.CalledProcessError:
        return []

    return [
        int(pid)
        for pid in output.split()
        if pid.isdigit() and int(pid) != os.getpid()
    ]

def find_tcp_port_pids(port: int, logger: Logger | None = None) -> list[int]:
    try:
        output = subprocess.check_output(
            ["fuser", "-n", "tcp", str(port)],
            stderr=subprocess.DEVNULL,
            text=True
        )
    except subprocess.CalledProcessError:
        return []
    except FileNotFoundError:
        _msg = f"Cannot clean TCP port {port}: the 'fuser' command is not installed."
        if logger:
            logger.error(_msg) #type: ignore
        else:
            print(_msg)
        return []

    return sorted({
        int(pid)
        for pid in output.split()
        if pid.isdigit() and int(pid) != os.getpid()
    })

def signal_pids(pids: list[int], sig: signal.Signals, logger: Logger | None = None) -> None:
    for pid in pids:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
        except PermissionError:
            _msg = f"Permission denied sending {sig.name} to PID {pid}."
            if logger:
                logger.error(_msg) #type: ignore
            else: 
                print(_msg)

def wait_for_tcp_port(port: int, timeout_sec: float) -> list[int]:
    deadline = time.monotonic() + timeout_sec
    remaining_pids = find_tcp_port_pids(port)

    while remaining_pids and time.monotonic() < deadline:
        time.sleep(0.2)
        remaining_pids = find_tcp_port_pids(port)

    return remaining_pids

def clear_tcp_port(port: int = 9090, logger: Logger | None = None) -> bool:
    remaining_pids = find_tcp_port_pids(port)
    if not remaining_pids:
        _msg = f"✅ TCP port {port} is free"
        if logger:
                logger.info(_msg) #type: ignore
        else: 
            print(_msg)
        return True

    for sig, timeout_sec in (
        (signal.SIGINT, 2.0),
        (signal.SIGTERM, 2.0),
        (signal.SIGKILL, 1.0),
    ):
        
        _msg = f"TCP port {port} still used by PID(s) {remaining_pids}; sending {sig.name}."
        if logger:
            logger.warn(_msg) #type: ignore
        else: 
            print(_msg)
        signal_pids(remaining_pids, sig)
        remaining_pids = wait_for_tcp_port(port, timeout_sec)
        if not remaining_pids:
            _msg = f"✅ TCP port {port} released"
            if logger:
                logger.info(_msg) #type: ignore
            else:
                print(_msg)
            return True

    _msg = f"TCP port {port} is still used by PID(s) {remaining_pids}."
    if logger:
        logger.error(_msg) #type: ignore
    else:
        print(_msg)
    return False
    

def stop_launch_environment(pattern: str, label: str, detach_port: int | None = None) -> None:
    launcher_pids = find_matching_pids(pattern)
    if launcher_pids:
        signal_pids(launcher_pids, signal.SIGINT)

    if detach_port is not None:
        # Give ROS launch time to stop its child processes gracefully before
        # escalating against any rosbridge instance still holding port 9090.
        wait_for_tcp_port(detach_port, 2.0) # 9090
        clear_tcp_port(detach_port)

def check_env_running() -> bool:

    if check_env([
        "run_platform_control.launch.py",
        "run_rosbridge.launch.py"
    ]):
        return True

    return False

def check_env_running_recovery() -> bool:

    if check_env([
        "run_z_recovery_control.launch.py",
        "run_rosbridge.launch.py"
    ]):
        return True

    return False

def check_env(launcher_names : list[str]) -> bool:
    
    found : list[bool] = [] 
    for launcher_name in launcher_names:
        launcher_pids = find_matching_pids(launcher_name)
        found.append(bool(launcher_pids))

    return all(found)

def check_env_running_stopped() -> bool:

    if check_env_stopped([
        "run_platform_control.launch.py",
          "run_rosbridge.launch.py"
    ]):
        return True
    
    return False
    
def check_env_running_recovery_stopped() -> bool:

    if check_env_stopped([
        "run_z_recovery_control.launch.py",
        "run_rosbridge.launch.py"
    ]):
        return True
    
    return False

def check_env_stopped(launcher_names : list[str]) -> bool:
    stopped : list[bool] = [] 
    for launcher_name in launcher_names:
        launcher_pids = find_matching_pids(launcher_name)
        stopped.append(not bool(launcher_pids))
    
    return all(stopped)
