# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

import os
import threading
import time
from typing import Any, Callable

from ethercat_msgs.srv import GetSlaveStates
from std_srvs.srv import Trigger

from plc_manager.plc_types import EthercatCheckState, State


class EthercatMonitor:
    def __init__(
        self,
        node: Any,
        service_group: Any,
        timer_group: Any,
        fsm_state_getter: Callable[[], State],
    ) -> None:
        self.node = node
        self._fsm_state_getter = fsm_state_getter

        self.node.declare_parameter('plc_slave_identifier', 'sickPLC')
        self.node.declare_parameter(
            'ethercat_startup_check_service',
            '/ethercat_slaves_status_check/check',
        )
        self.node.declare_parameter(
            'ethercat_slave_state_services',
            ['/tecnobody/get_slave_states_master1'],
        )
        self.node.declare_parameter('ethercat_state_poll_period', 1.0)
        self.node.declare_parameter('ethercat_state_stale_after', 5.0)
        self.node.declare_parameter('ethercat_state_response_timeout', 3.0)

        self.startup_check_service_name = self._get_startup_check_service_name()
        self.startup_check_client = self.node.create_client(  # type: ignore
            Trigger,
            self.startup_check_service_name,
            callback_group=service_group,
        )
        self.startup_check_state = EthercatCheckState.IDLE
        self.startup_check_requested = True

        self.check_state = EthercatCheckState.IDLE
        self._cache_lock = threading.Lock()
        self._slave_names: list[str] = []
        self._slave_states: list[str] = []
        self._last_update_monotonic: float | None = None
        self._last_error: str | None = None
        self._pending_requests: dict[Any, str] = {}
        self._poll_started_at: float | None = None
        self._poll_slave_names: list[str] = []
        self._poll_slave_states: list[str] = []
        self._poll_errors: list[str] = []
        self._state_stale_after = float(
            self.node.get_parameter('ethercat_state_stale_after').value  # type: ignore
        )
        self._state_response_timeout = float(
            self.node.get_parameter('ethercat_state_response_timeout').value  # type: ignore
        )
        self._slave_state_clients = {
            service_name: self.node.create_client(  # type: ignore
                GetSlaveStates,
                service_name,
                callback_group=service_group,
            )
            for service_name in self._get_slave_state_service_names()
        }
        self.state_timer = self.node.create_timer(
            float(self.node.get_parameter('ethercat_state_poll_period').value),  # type: ignore
            self.poll_slave_states,
            callback_group=timer_group,
            autostart=False,
        )

    def startup_status_ok(self) -> bool:
        return self.startup_check_state == EthercatCheckState.READY

    def run_startup_check(self, executor: Any) -> bool:
        if os.environ.get('PLC_MANAGER_SKIP_ETHERCAT', '0') == '1':
            self.startup_check_state = EthercatCheckState.READY
            self._last_error = (
                'Skipped EtherCAT startup check '
                '(PLC_MANAGER_SKIP_ETHERCAT=1).'
            )
            return True
        if not self.startup_check_requested:
            return True

        if not self.startup_check_client.wait_for_service(timeout_sec=5.0):  # type: ignore
            self.node.get_logger().error(  # type: ignore
                "EtherCAT status check service not available. "
                f"Ensure {self.startup_check_service_name} "
                "is running."
            )
            return False

        future = self.startup_check_client.call_async(Trigger.Request())  # type: ignore
        self.startup_check_state = EthercatCheckState.PENDING

        executor.spin_until_future_complete(
            future,  # type: ignore
            timeout_sec=5.0,
        )

        if future.done() and future.result() is not None:  # type: ignore
            startup_check_result = future.result()  # type: ignore
            self.startup_check_state = (
                EthercatCheckState.READY
                if startup_check_result.success
                else EthercatCheckState.FAILED
            )
            if not startup_check_result.success:
                self.node.get_logger().warning(  # type: ignore
                    "EtherCAT startup check failed: "
                    f"{startup_check_result.message}"
                )
        else:
            self.startup_check_state = EthercatCheckState.FAILED
            self.node.get_logger().warning(  # type: ignore
                "EtherCAT startup check timed out."
            )

        self.startup_check_requested = not self.startup_status_ok()
        return True

    def start_runtime_polling(self) -> None:
        self.state_timer.reset()

    def _get_startup_check_service_name(self) -> str:
        service_name = str(
            self.node.get_parameter('ethercat_startup_check_service').value  # type: ignore
        ).strip()
        return service_name or '/ethercat_slaves_status_check/check'

    def _get_slave_state_service_names(self) -> list[str]:
        raw_service_names = self.node.get_parameter(
            'ethercat_slave_state_services'
        ).value  # type: ignore

        if raw_service_names is None:
            service_names = []
        elif isinstance(raw_service_names, str):
            service_names = [
                name.strip()
                for name in raw_service_names.split(',')
                if name.strip()
            ]
        else:
            service_names = [
                str(name).strip()
                for name in raw_service_names
                if str(name).strip()
            ]

        return service_names or ['/tecnobody/get_slave_states_master1']

    def _plc_slave_identifier(self) -> str:
        return str(self.node.get_parameter('plc_slave_identifier').value)  # type: ignore

    def _cache_stale_locked(self) -> bool:
        if os.environ.get('PLC_MANAGER_SKIP_ETHERCAT', '0') == '1':
            return False

        if self._last_update_monotonic is None:
            return True

        return (
            time.monotonic() - self._last_update_monotonic
            > self._state_stale_after
        )

    def _find_plc_slave_locked(self) -> tuple[str | None, str | None]:
        plc_identifier = self._plc_slave_identifier()
        for slave_name, slave_state in zip(
            self._slave_names,
            self._slave_states,
        ):
            if plc_identifier in slave_name:
                return slave_name, slave_state

        return None, None

    def _visible_slave_pairs_locked(self) -> list[tuple[str, str]]:
        return list(zip(self._slave_names, self._slave_states))

    def _refresh_check_state_locked(self) -> None:
        if os.environ.get('PLC_MANAGER_SKIP_ETHERCAT', '0') == '1':
            self.check_state = EthercatCheckState.READY
            self._last_error = (
                'Skipped EtherCAT health check '
                '(PLC_MANAGER_SKIP_ETHERCAT=1).'
            )
            return

        if not self._slave_names:
            self.check_state = EthercatCheckState.FAILED
            self._last_error = 'EtherCAT GetSlaveStates returned no slaves.'
            return

        if len(self._slave_names) != len(self._slave_states):
            self.check_state = EthercatCheckState.FAILED
            self._last_error = (
                'EtherCAT GetSlaveStates returned mismatched slave names/states.'
            )
            return

        non_operational_slaves = [
            f'{slave_name} => {slave_state}'
            for slave_name, slave_state in zip(
                self._slave_names,
                self._slave_states,
            )
            if slave_state != 'OP'
        ]

        if not non_operational_slaves:
            self.check_state = EthercatCheckState.READY
            if self._poll_errors:
                self._last_error = '; '.join(self._poll_errors)
            else:
                self._last_error = None
            return

        self.check_state = EthercatCheckState.FAILED
        self._last_error = (
            'EtherCAT slaves are not operational: '
            + '; '.join(non_operational_slaves)
        )

    def _mark_check_failed(self, message: str) -> None:
        with self._cache_lock:
            self.check_state = EthercatCheckState.FAILED
            self._last_error = message

        self.node.get_logger().warning(message, throttle_duration_sec=5.0)  # type: ignore

    def _check_request_timeout(self) -> None:
        with self._cache_lock:
            if not self._pending_requests:
                return

            if self._poll_started_at is None:
                return

            if (
                time.monotonic() - self._poll_started_at
                <= self._state_response_timeout
            ):
                return

            timed_out_services = ', '.join(
                self._pending_requests.values()
            )
            self._pending_requests.clear()
            self._poll_started_at = None
            self.check_state = EthercatCheckState.FAILED
            self._last_error = (
                'Timed out waiting for EtherCAT state services: '
                f'{timed_out_services}'
            )

        message = self._last_error or 'EtherCAT state request timed out.'
        self.node.get_logger().warning(message, throttle_duration_sec=5.0)  # type: ignore

    def poll_slave_states(self) -> None:
        if os.environ.get('PLC_MANAGER_SKIP_ETHERCAT', '0') == '1':
            with self._cache_lock:
                self._last_update_monotonic = time.monotonic()
                self._slave_names = []
                self._slave_states = []
                self._refresh_check_state_locked()
            return

        if self._fsm_state_getter() not in (State.RUNNING, State.RUNNING_RECOVERY):
            with self._cache_lock:
                self._pending_requests.clear()
                self._poll_started_at = None
                self._slave_names = []
                self._slave_states = []
                self._last_update_monotonic = None
                self.check_state = EthercatCheckState.IDLE
                self._last_error = (
                    'Runtime EtherCAT telemetry waits for START.'
                )
            return

        with self._cache_lock:
            if self._pending_requests:
                should_check_timeout = True
            else:
                should_check_timeout = False

        if should_check_timeout:
            self._check_request_timeout()
            return

        ready_clients = [
            (service_name, client)
            for service_name, client in self._slave_state_clients.items()
            if client.service_is_ready()  # type: ignore
        ]

        if not ready_clients:
            service_names = ', '.join(self._slave_state_clients.keys())
            self._mark_check_failed(
                'No configured EtherCAT GetSlaveStates services are ready: '
                f'{service_names}'
            )
            return

        with self._cache_lock:
            self.check_state = EthercatCheckState.PENDING
            self._poll_started_at = time.monotonic()
            self._poll_slave_names = []
            self._poll_slave_states = []
            self._poll_errors = []

        started_requests = 0
        for service_name, client in ready_clients:
            try:
                future = client.call_async(GetSlaveStates.Request())  # type: ignore
            except Exception as exception:  # noqa: BLE001
                with self._cache_lock:
                    self._poll_errors.append(
                        f'{service_name}: call failed ({exception})'
                    )
                continue

            with self._cache_lock:
                self._pending_requests[future] = service_name

            future.add_done_callback(self._on_slave_states_response)  # type: ignore
            started_requests += 1

        if started_requests == 0:
            with self._cache_lock:
                errors = '; '.join(self._poll_errors)
            self._mark_check_failed(
                errors or 'No EtherCAT GetSlaveStates request could be started.'
            )

    def _on_slave_states_response(self, future: Any) -> None:
        with self._cache_lock:
            service_name = self._pending_requests.pop(future, None)

        if service_name is None:
            return

        try:
            response = future.result()
            slave_names = [str(name) for name in response.slave_names]
            slave_states = [str(state) for state in response.slave_states]
        except Exception as exception:  # noqa: BLE001
            slave_names = []
            slave_states = []
            error = f'{service_name}: call failed ({exception})'
        else:
            error = None

        with self._cache_lock:
            if error is not None:
                self._poll_errors.append(error)
            else:
                self._poll_slave_names.extend(slave_names)
                self._poll_slave_states.extend(slave_states)

            if self._pending_requests:
                return

            self._finalize_poll_locked()

    def _finalize_poll_locked(self) -> None:
        self._poll_started_at = None

        if not self._poll_slave_names:
            self.check_state = EthercatCheckState.FAILED
            self._last_error = (
                '; '.join(self._poll_errors)
                or 'EtherCAT GetSlaveStates returned no slaves.'
            )
            return

        self._slave_names = list(self._poll_slave_names)
        self._slave_states = list(self._poll_slave_states)
        self._last_update_monotonic = time.monotonic()
        self._refresh_check_state_locked()

    def status_payload(self) -> dict[str, object]:
        with self._cache_lock:
            last_update_age = (
                None
                if self._last_update_monotonic is None
                else max(0.0, time.monotonic() - self._last_update_monotonic)
            )
            plc_name, plc_state = self._find_plc_slave_locked()
            stale = self._cache_stale_locked()
            visible_pairs = self._visible_slave_pairs_locked()

            return {
                "ok": (
                    self.check_state == EthercatCheckState.READY
                    and not stale
                ),
                "state": self.check_state.name,
                "stale": stale,
                "last_update_age_s": (
                    None if last_update_age is None else round(last_update_age, 3)
                ),
                "plc_slave_identifier": self._plc_slave_identifier(),
                "plc_slave": (
                    None
                    if plc_name is None
                    else {"name": plc_name, "state": plc_state}
                ),
                "service_names": list(self._slave_state_clients.keys()),
                "slaves": [
                    {"name": slave_name, "state": slave_state}
                    for slave_name, slave_state in visible_pairs
                ],
                "last_error": self._last_error,
            }
