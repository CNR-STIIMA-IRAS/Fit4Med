#!/usr/bin/env python3
from dataclasses import dataclass, field
import os
import signal
import subprocess
import threading
import time
from typing import Any, Sequence

from rclpy.impl.rcutils_logger import RcutilsLogger as Logger


@dataclass
class ControllerStatusCheckResult:
    controller_manager_name: str
    expected_active: list[str]
    expected_inactive: list[str]
    observed_states: dict[str, str] = field(default_factory=dict)
    missing_controllers: list[str] = field(default_factory=list)
    wrong_state_controllers: dict[str, str] = field(default_factory=dict)
    error: str | None = None

    @property
    def ok(self) -> bool:
        return (
            self.error is None
            and not self.missing_controllers
            and not self.wrong_state_controllers
        )

    def __bool__(self) -> bool:
        return self.ok

    @property
    def message(self) -> str:
        if self.ok:
            return (
                f"Controller manager {self.controller_manager_name} status OK "
                f"(active={self.expected_active}, inactive={self.expected_inactive})."
            )

        problems: list[str] = []
        if self.error:
            problems.append(self.error)
        if self.missing_controllers:
            problems.append(
                "missing: " + ", ".join(sorted(self.missing_controllers))
            )
        if self.wrong_state_controllers:
            wrong_states = ", ".join(
                f"{name}={state}"
                for name, state in sorted(self.wrong_state_controllers.items())
            )
            problems.append(f"wrong state: {wrong_states}")
        return (
            f"Controller manager {self.controller_manager_name} status not OK "
            f"({'; '.join(problems)})."
        )


def _normalize_controller_manager_name(controller_manager_name: str) -> str:
    normalized_name = controller_manager_name.strip()
    if not normalized_name:
        normalized_name = "controller_manager"
    if not normalized_name.startswith("/"):
        normalized_name = f"/{normalized_name}"
    return normalized_name.rstrip("/") or "/controller_manager"


def _controller_states_from_response(controllers: Sequence[Any]) -> dict[str, str]:
    states: dict[str, str] = {}
    for controller in controllers:
        if isinstance(controller, dict):
            name = controller.get("name")
            state = controller.get("state")
        else:
            name = getattr(controller, "name", None)
            state = getattr(controller, "state", None)

        if name is not None and state is not None:
            states[str(name)] = str(state)
    return states


def _controller_name_list(controllers: Sequence[str] | None) -> list[str]:
    return [str(controller) for controller in (controllers or [])]


def _destroy_client_if_possible(node: Any, client: Any) -> None:
    destroy_client = getattr(node, "destroy_client", None)
    if callable(destroy_client):
        destroy_client(client)


def check_controller_states(
    controllers: Sequence[Any],
    controller_manager_name: str,
    active_controllers: Sequence[str] | None = None,
    inactive_controllers: Sequence[str] | None = None,
) -> ControllerStatusCheckResult:
    """Check a controller list response against expected active/inactive states."""
    expected_active = _controller_name_list(active_controllers)
    expected_inactive = _controller_name_list(inactive_controllers)
    observed_states = _controller_states_from_response(controllers)
    duplicated_expectations = sorted(
        set(expected_active) & set(expected_inactive)
    )

    expected_states = {
        controller_name: "active"
        for controller_name in expected_active
    }
    expected_states.update({
        controller_name: "inactive"
        for controller_name in expected_inactive
    })

    missing_controllers: list[str] = []
    wrong_state_controllers: dict[str, str] = {}

    for controller_name, expected_state in expected_states.items():
        observed_state = observed_states.get(controller_name)
        if observed_state is None:
            missing_controllers.append(controller_name)
        elif observed_state != expected_state:
            wrong_state_controllers[controller_name] = observed_state

    error = None
    if duplicated_expectations:
        error = (
            "controllers cannot be expected active and inactive at the same time: "
            + ", ".join(duplicated_expectations)
        )

    return ControllerStatusCheckResult(
        controller_manager_name=_normalize_controller_manager_name(
            controller_manager_name
        ),
        expected_active=expected_active,
        expected_inactive=expected_inactive,
        observed_states=observed_states,
        missing_controllers=missing_controllers,
        wrong_state_controllers=wrong_state_controllers,
        error=error,
    )


def check_controller_manager_status(
    node: Any,
    controller_manager_name: str,
    active_controllers: Sequence[str] | None = None,
    inactive_controllers: Sequence[str] | None = None,
    timeout_sec: float = 3.0,
    logger: Logger | None = None,
) -> ControllerStatusCheckResult:
    """Call controller_manager/list_controllers and check expected states."""
    from controller_manager_msgs.srv import ListControllers
    import rclpy

    normalized_name = _normalize_controller_manager_name(controller_manager_name)
    service_name = f"{normalized_name}/list_controllers"
    client = node.create_client(ListControllers, service_name)

    try:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            result = ControllerStatusCheckResult(
                controller_manager_name=normalized_name,
                expected_active=_controller_name_list(active_controllers),
                expected_inactive=_controller_name_list(inactive_controllers),
                error=(
                    f"service {service_name} not available after "
                    f"{timeout_sec:.1f}s"
                ),
            )
            if logger:
                logger.warn(result.message) #type: ignore
            return result

        future = client.call_async(ListControllers.Request())
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if not future.done():
            result = ControllerStatusCheckResult(
                controller_manager_name=normalized_name,
                expected_active=_controller_name_list(active_controllers),
                expected_inactive=_controller_name_list(inactive_controllers),
                error=(
                    f"service {service_name} did not respond after "
                    f"{timeout_sec:.1f}s"
                ),
            )
            if logger:
                logger.warn(result.message) #type: ignore
            return result

        try:
            response = future.result()
        except Exception as exception:  # noqa: BLE001
            result = ControllerStatusCheckResult(
                controller_manager_name=normalized_name,
                expected_active=_controller_name_list(active_controllers),
                expected_inactive=_controller_name_list(inactive_controllers),
                error=f"service {service_name} call failed: {exception}",
            )
            if logger:
                logger.error(result.message) #type: ignore
            return result

        result = check_controller_states(
            response.controller,
            normalized_name,
            active_controllers,
            inactive_controllers,
        )
        if logger:
            if result.ok:
                logger.info(result.message) #type: ignore
            else:
                logger.warn(result.message) #type: ignore
        return result
    finally:
        _destroy_client_if_possible(node, client)


class ControllerManagerStatusMonitor:
    def __init__(
        self,
        node: Any,
        controller_manager_name: str,
        active_controllers: Sequence[str] | None = None,
        inactive_controllers: Sequence[str] | None = None,
        request_period_sec: float = 0.2,
        stale_after_sec: float = 3.0,
        callback_group: Any | None = None,
        logger: Logger | None = None,
    ) -> None:
        from controller_manager_msgs.srv import ListControllers

        self._node = node
        self._controller_manager_name = _normalize_controller_manager_name(
            controller_manager_name
        )
        self._active_controllers = _controller_name_list(active_controllers)
        self._inactive_controllers = _controller_name_list(inactive_controllers)
        self._request_period_sec = request_period_sec
        self._stale_after_sec = stale_after_sec
        self._logger = logger
        self._lock = threading.RLock()
        self._last_logged_message: str | None = None
        self._last_request_monotonic: float | None = None
        self._last_response_monotonic: float | None = None
        self._pending_future: Any | None = None
        self._last_result = ControllerStatusCheckResult(
            controller_manager_name=self._controller_manager_name,
            expected_active=self._active_controllers,
            expected_inactive=self._inactive_controllers,
            error="controller status not checked yet",
        )
        self._list_controllers_type = ListControllers
        self._client = node.create_client(
            ListControllers,
            f"{self._controller_manager_name}/list_controllers",
            callback_group=callback_group,
        )

    @property
    def last_result(self) -> ControllerStatusCheckResult:
        with self._lock:
            return self._last_result

    @property
    def message(self) -> str:
        with self._lock:
            return self._current_result_locked().message

    def ok(self) -> bool:
        self.request_update_if_needed()
        with self._lock:
            return self._current_result_locked().ok

    def request_update_if_needed(self, force: bool = False) -> None:
        with self._lock:
            self._collect_finished_request_locked()

            if self._pending_future is not None:
                return

            now = time.monotonic()
            if (
                not force
                and self._last_request_monotonic is not None
                and now - self._last_request_monotonic < self._request_period_sec
            ):
                return

            if not self._client.service_is_ready():
                self._last_request_monotonic = now
                if self._last_response_monotonic is None:
                    self._last_result = self._error_result(
                        "controller manager list_controllers service is not ready"
                    )
                return

            self._last_request_monotonic = now
            try:
                future = self._client.call_async(
                    self._list_controllers_type.Request()
                )
            except Exception as exception:  # noqa: BLE001
                self._last_result = self._error_result(
                    "controller manager list_controllers request failed: "
                    f"{exception}"
                )
                return

            self._pending_future = future
            future.add_done_callback(self._on_list_controllers_response)

    def reset(self) -> None:
        with self._lock:
            self._pending_future = None
            self._last_request_monotonic = None
            self._last_response_monotonic = None
            self._last_logged_message = None
            self._last_result = self._error_result("controller status not checked yet")

    def destroy(self) -> None:
        with self._lock:
            self._pending_future = None
        _destroy_client_if_possible(self._node, self._client)

    def _collect_finished_request_locked(self) -> None:
        if self._pending_future is not None and self._pending_future.done():
            self._process_response_locked(self._pending_future)

    def _on_list_controllers_response(self, future: Any) -> None:
        with self._lock:
            self._process_response_locked(future)

    def _process_response_locked(self, future: Any) -> None:
        if future is not self._pending_future:
            return

        self._pending_future = None
        self._last_response_monotonic = time.monotonic()
        try:
            response = future.result()
        except Exception as exception:  # noqa: BLE001
            self._last_result = self._error_result(
                f"controller manager list_controllers response failed: {exception}"
            )
            self._log_result()
            return

        self._last_result = check_controller_states(
            response.controller,
            self._controller_manager_name,
            self._active_controllers,
            self._inactive_controllers,
        )
        self._log_result()

    def _current_result_locked(self) -> ControllerStatusCheckResult:
        if self._last_response_monotonic is None:
            return self._last_result

        age_sec = time.monotonic() - self._last_response_monotonic
        if age_sec <= self._stale_after_sec:
            return self._last_result

        return self._error_result(
            f"controller status is stale ({age_sec:.1f}s old)"
        )

    def _error_result(self, error: str) -> ControllerStatusCheckResult:
        return ControllerStatusCheckResult(
            controller_manager_name=self._controller_manager_name,
            expected_active=self._active_controllers,
            expected_inactive=self._inactive_controllers,
            error=error,
        )

    def _log_result(self) -> None:
        if not self._logger:
            return
        if self._last_logged_message == self._last_result.message:
            return

        if self._last_result.ok:
            self._logger.info(self._last_result.message) #type: ignore
        else:
            self._logger.warn(self._last_result.message) #type: ignore
        self._last_logged_message = self._last_result.message


def make_platform_controller_status_monitor(
    node: Any,
    callback_group: Any | None = None,
    logger: Logger | None = None,
) -> ControllerManagerStatusMonitor:
    return ControllerManagerStatusMonitor(
        node=node,
        controller_manager_name="controller_manager",
        active_controllers=[
            "state_controller",
            "joint_state_broadcaster",
            "ft_sensor_command_broadcaster",
            "remapping_controller",
            "forward_position_controller",
            "joint_trajectory_controller",
        ],
        inactive_controllers=[
            "forward_velocity_controller",
            "admittance_controller",
            "go_to_start_controller",
        ],
        callback_group=callback_group,
        logger=logger,
    )


def make_recovery_controller_status_monitor(
    node: Any,
    callback_group: Any | None = None,
    logger: Logger | None = None,
) -> ControllerManagerStatusMonitor:
    return ControllerManagerStatusMonitor(
        node=node,
        controller_manager_name="controller_manager",
        active_controllers=[
            "state_controller",
            "joint_state_broadcaster",
            "ft_sensor_command_broadcaster",
            "forward_velocity_controller",
        ],
        inactive_controllers=[
        ],
        callback_group=callback_group,
        logger=logger,
    )


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


def check_env(launcher_names : list[str]) -> bool:
    
    found : list[bool] = [] 
    for launcher_name in launcher_names:
        launcher_pids = find_matching_pids(launcher_name)
        found.append(bool(launcher_pids))

    return all(found)


def check_env_stopped(launcher_names : list[str]) -> bool:
    stopped : list[bool] = [] 
    for launcher_name in launcher_names:
        launcher_pids = find_matching_pids(launcher_name)
        stopped.append(not bool(launcher_pids))
    
    return all(stopped)
