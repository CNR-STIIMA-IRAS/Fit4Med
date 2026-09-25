# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from PyQt5.QtCore import QThread, QObject, pyqtSignal, pyqtSlot
import json
import socket
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional, Tuple

################################################
#
#
#
################################################
import os
import sys
import time

def _disable_udp_connreset(sock: socket.socket) -> None:
    """Windows: stop ICMP port-unreachable surfacing as recvfrom() errors.

    socket.ioctl() only accepts a few SIO_* codes, so go through WSAIoctl.
    """
    if sys.platform != "win32":
        return
    import ctypes
    from ctypes import wintypes
    SIO_UDP_CONNRESET = 0x9800000C
    wsa_ioctl = ctypes.windll.ws2_32.WSAIoctl
    wsa_ioctl.argtypes = [ctypes.c_size_t, wintypes.DWORD, ctypes.c_void_p, wintypes.DWORD,
                          ctypes.c_void_p, wintypes.DWORD, ctypes.POINTER(wintypes.DWORD),
                          ctypes.c_void_p, ctypes.c_void_p]
    flag = wintypes.BOOL(False)
    returned = wintypes.DWORD(0)
    if wsa_ioctl(sock.fileno(), SIO_UDP_CONNRESET, ctypes.byref(flag), ctypes.sizeof(flag),
                 None, 0, ctypes.byref(returned), None, None) != 0:
        print(f"[UdpServer] SIO_UDP_CONNRESET failed (WSA error {ctypes.windll.ws2_32.WSAGetLastError()})")

class UdpServer(QObject):
    message_received = pyqtSignal(bytes, tuple)  # data, addr
    bind_failed = pyqtSignal(str)

    MIN_EMIT_PERIOD_S = 0.1  # repeats of an unchanged status reach the GUI at most at 10 Hz

    def __init__(self, host:str="0.0.0.0", port:int=5005, parent:Any=None):
        super().__init__(parent)
        self.host = host
        self.port = port
        self.sock = None
        self._running = False
        self._last_addr = None  # ultimo client da cui ho ricevuto
        print(f"[UdpServer] Initialized on {self.host}:{self.port}")

    def start(self):
        """Start UDP server (blocking, run in a separate QThread)."""
        if self._running:
            print("[UdpServer] Already running")
            return
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Leftovers holding the port are killed by ps_scripts/fmrr_gui.ps1 before launch.
        try:
            self.sock.bind((self.host, self.port))
        except OSError as e:
            # This runs as a slot on its own QThread: an uncaught exception here
            # is silently swallowed by Qt, leaving the UDP layer dead with no
            # visible error -- the rest of the GUI keeps running normally, but
            # nothing from the embedded controller ever arrives again. Surface
            # it instead so the failure is visible and the socket is released.
            print(f"[UdpServer] Failed to bind UDP port {self.port}: {e}")
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
            self.bind_failed.emit(str(e))
            return
        _disable_udp_connreset(self.sock)
        self._running = True
        sock = self.sock
        # Signal-rate limit: every packet is read here, but the GUI thread is
        # handed a new one only when the FSM state/pending changes (at once)
        # or, for repeats of the same state, the latest one every
        # MIN_EMIT_PERIOD_S. A busy GUI thread thus never accumulates a queue.
        last_emit = 0.0
        last_key = None
        held = None  # newest not-yet-emitted repeat
        try:
            while self._running:
                now = time.monotonic()
                wait = max(0.01, self.MIN_EMIT_PERIOD_S - (now - last_emit)) if held else 0.2
                sock.settimeout(wait)
                try:
                    data, addr = sock.recvfrom(4096)
                except socket.timeout:
                    data = None
                except ConnectionResetError as e:
                    # Windows reports an ICMP "port unreachable" for one of our
                    # earlier sendto() here; the socket is still fine. Leaving
                    # the loop used to keep 5005 bound with nobody reading it.
                    print(f"[UdpServer] Ignoring UDP connection reset: {e}")
                    continue
                except OSError as e:
                    if not self._running:
                        break  # socket closed by stop()
                    print(f"[UdpServer] OSError: {e}")
                    time.sleep(0.2)
                    continue

                now = time.monotonic()
                if data is not None:
                    self._last_addr = addr
                    key = self._status_key(data)
                    if key != last_key or now - last_emit >= self.MIN_EMIT_PERIOD_S:
                        self.message_received.emit(data, addr)
                        last_emit, last_key, held = now, key, None
                    else:
                        held = (data, addr)
                elif held is not None and now - last_emit >= self.MIN_EMIT_PERIOD_S:
                    self.message_received.emit(*held)
                    last_emit, held = now, None
        finally:
            self._running = False
            try:
                sock.close()
            except OSError:
                pass

    @staticmethod
    def _status_key(data: bytes) -> Any:
        """What makes a status 'new' (same key as plc_manager's UdpClient)."""
        try:
            payload = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return data  # legacy plain-text status
        if not isinstance(payload, dict):
            return data
        pending = payload.get("pending")
        if not isinstance(pending, dict):
            return (payload.get("state"), None, None, None)
        return (payload.get("state"), pending.get("event"), pending.get("source"), pending.get("target"))

    def stop(self):
        """Stop the server."""
        print("[UdpServer] Stopping server...")
        self._running = False
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def send_response(self, message: bytes, addr: Optional[Tuple[Any, ...]] =None):
        """Send response to client."""
        sock = self.sock  # stop() may clear self.sock from another thread
        if not self._running or sock is None:
            return
        if addr is None:
            addr = self._last_addr
        if addr:
            try:
                sock.sendto(message, addr)
            except OSError as e:
                # Called from GUI slots: an uncaught exception there aborts the process.
                print(f"[UdpServer] sendto {addr} failed: {e}")
            # print("[UdpServer] Sent response to {}: {}".format(addr, message))

################################################
#
#
#
################################################
class UdpCommunicationManager(QObject):

    start_ros_communication : pyqtSignal = pyqtSignal()
    stop_ros_communication : pyqtSignal = pyqtSignal()
    udp_message_received : pyqtSignal = pyqtSignal(bytes, tuple)
    plc_status_payload_received : pyqtSignal = pyqtSignal(dict)
    udp_bind_failed : pyqtSignal = pyqtSignal(str)

    # Replies to plc_manager. They describe the GUI's ROS state and are
    # repeated on every status packet, so a lost or misdirected reply is
    # corrected by the next one (plc_manager keeps only the last received).
    ROS_CONNECTED = b"ROS_CONNECTED"
    ROS_DISCONNECTED = b"ROS_DISCONNECTED"
    ROS_CONNECTION_FAILED = b"ROS_CONNECTION_FAILED"
    PLC_RUNNING_STATES = ('RUNNING', 'RUNNING_RECOVERY')

    DEFAULT_PLC_STATES: Dict[str, Any] = {
        's_input.0': False,
        'estop': False,
        'reset': False,
        'manual_switch_pressed': False,
        's_input.5': False,
        's_input.6': False,
        's_input.7': False,
        's_input.8': False,
    }

    def __init__(self, remote_ip: str, remote_port: int, number_of_ec_slaves: int = 0): #port=5005
        super().__init__()

        self.start_ros_communication_emitted  : bool = False
        self.stop_ros_communication_emitted  : bool = False
        self._ros_communication_active_checker: Optional[Callable[[], bool]] = None
        self._ros_reply: bytes = self.ROS_DISCONNECTED

        self.remote_ip = remote_ip
        self.remote_port = remote_port
        self.number_of_ec_slaves = number_of_ec_slaves
        self.roslib_first_time_connection = True
        self._last_plc_status_payload: Dict[str, Any] = {}
        self._last_plc_state: Optional[str] = None
        self._last_plc_pending: Optional[Dict[str, Any]] = None
        self._last_udp_received_monotonic: Optional[float] = None
        self._last_udp_received_wall_time: Optional[str] = None
        self._last_plc_status_received_monotonic: Optional[float] = None
        self._last_plc_status_received_wall_time: Optional[str] = None
        self._last_udp_display_message = "No UDP message received yet"
        self._last_ethercat_payload: Dict[str, Any] = {}
        self._plc_inputs: Dict[str, Any] = dict(self.DEFAULT_PLC_STATES)
        self._plc_outputs: Dict[str, Any] = {}
        self._plc_input_items: List[Tuple[str, Any]] = []
        self._plc_output_items: List[Tuple[str, Any]] = []
        self._cached_slave_names: List[str] = self._fit_slave_values([])
        self._cached_slave_states: List[str] = self._fit_slave_values([])
        self._udp_bind_failure_reason: Optional[str] = None
        self.udp_thread = QThread()
        self.server = UdpServer(port=remote_port)
        self.server.moveToThread(self.udp_thread)
        self.udp_thread.started.connect(self.server.start)

        self.server.message_received.connect(self.onUdpMessageReceived)
        self.server.bind_failed.connect(self._onUdpBindFailed)
        self.udp_thread.start()

    def setRosCommunicationActiveChecker(self, checker: Callable[[], bool]) -> None:
        self._ros_communication_active_checker = checker

    def shutdown(self, wait_msec: int = 2000) -> None:
        """Stop the UDP socket and wait for the worker thread to exit."""
        if not self.udp_thread.isRunning():
            return

        self.server.stop()
        self.udp_thread.quit()
        if not self.udp_thread.wait(wait_msec):
            print("[UdpServer] UDP thread did not stop cleanly.")

    @pyqtSlot(bytes, tuple)
    def onUdpMessageReceived(self, data: bytes, addr: str) -> None:
        self._cache_udp_received_time()
        self.udpMessageReceived(data)
        self.server.send_response(self._ros_reply)
        self.udp_message_received.emit(data, addr)

    @pyqtSlot(str)
    def _onUdpBindFailed(self, reason: str) -> None:
        self._udp_bind_failure_reason = reason
        print(f"[UdpServer] UDP layer is down, no status from the embedded controller will arrive: {reason}")
        self.udp_bind_failed.emit(reason)

    def isUdpBindFailed(self) -> bool:
        return self._udp_bind_failure_reason is not None

    def getUdpBindFailureReason(self) -> Optional[str]:
        return self._udp_bind_failure_reason

    def _set_ros_reply(self, reply: bytes) -> None:
        self._ros_reply = reply
        self.server.send_response(reply)

    def _is_plc_running(self) -> bool:
        return self._last_plc_state in self.PLC_RUNNING_STATES

    def onResetRosCommunication(self) -> None:
        if self._ros_reply == self.ROS_CONNECTION_FAILED and self._is_plc_running():
            # plc_manager has not acted on the failure yet (FAIL leaves
            # RUNNING): a DISCONNECTED now could hide it. udpMessageReceived
            # switches to DISCONNECTED once the PLC has left RUNNING.
            self.server.send_response(self._ros_reply)
        else:
            self._set_ros_reply(self.ROS_DISCONNECTED)
        self.start_ros_communication_emitted = False
        # self.stop_ros_communication_emitted = False

    @pyqtSlot()
    def onRosCommunicationEstablished(self) -> None:
        print("[UdpServer] PLC Communication Established.")
        self._set_ros_reply(self.ROS_CONNECTED)

    @pyqtSlot()
    def onRosCommunicationFailed(self) -> None:
        print("[UdpServer] ROS communication failed.")
        self._set_ros_reply(self.ROS_CONNECTION_FAILED)
        self.start_ros_communication_emitted = False

    def requestRosCommunicationStop(self, reason: str) -> None:
        print(f"[UdpServer] {reason}")
        if not self.stop_ros_communication_emitted:
            self.stop_ros_communication.emit()
            self.stop_ros_communication_emitted = True

    def _is_ros_communication_inactive(self) -> bool:
        if self._ros_communication_active_checker is None:
            return False

        try:
            return not self._ros_communication_active_checker()
        except Exception as exc:
            print(f"[UdpServer] ROS communication activity check failed: {exc}")
            return True

    @staticmethod
    def _format_wall_time() -> str:
        return datetime.now().strftime("%H:%M:%S")

    def _cache_udp_received_time(self) -> None:
        self._last_udp_received_monotonic = time.monotonic()
        self._last_udp_received_wall_time = self._format_wall_time()

    def _fit_slave_values(self, values: List[str]) -> List[str]:
        if self.number_of_ec_slaves <= 0:
            return list(values)

        fitted_values = list(values[:self.number_of_ec_slaves])
        fitted_values.extend(['n/a'] * (self.number_of_ec_slaves - len(fitted_values)))
        return fitted_values

    @staticmethod
    def _interface_items(payload: Any) -> Optional[List[Tuple[str, Any]]]:
        if not isinstance(payload, dict):
            return None

        interface_names = payload.get("interface_names")
        values = payload.get("values")
        if not isinstance(interface_names, list) or not isinstance(values, list):
            return None

        items: List[Tuple[str, Any]] = []
        for name, value in zip(interface_names, values):
            if isinstance(name, str):
                items.append((name, value))
        return items

    @staticmethod
    def _value_as_bool(value: Any) -> bool:
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _cache_ethercat_payload(self, ethercat_payload: Any) -> None:
        if not isinstance(ethercat_payload, dict):
            return

        slave_names: List[str] = []
        slave_states: List[str] = []
        slaves = ethercat_payload.get("slaves")
        if isinstance(slaves, list):
            for slave in slaves:
                if not isinstance(slave, dict):
                    continue
                slave_name = slave.get("name")
                slave_state = slave.get("state")
                slave_names.append(str(slave_name) if slave_name is not None else "n/a")
                slave_states.append(str(slave_state) if slave_state is not None else "n/a")

        self._last_ethercat_payload = ethercat_payload
        self._cached_slave_names = self._fit_slave_values(slave_names)
        self._cached_slave_states = self._fit_slave_values(slave_states)

    def _cache_plc_status_payload(
        self,
        payload: Dict[str, Any],
        state: str,
        pending: Optional[Dict[str, Any]],
    ) -> None:
        self._last_plc_status_payload = payload
        self._last_plc_state = state
        self._last_plc_pending = pending
        self._last_plc_status_received_monotonic = time.monotonic()
        self._last_plc_status_received_wall_time = self._format_wall_time()
        self._last_udp_display_message = self._format_plc_status_display_message(state, pending)
        self._cache_ethercat_payload(payload.get("ethercat"))

        plc_inputs = self._interface_items(payload.get("plc_inputs"))
        if plc_inputs is not None:
            self._plc_input_items = plc_inputs
            next_inputs = dict(self.DEFAULT_PLC_STATES)
            next_inputs.update(dict(plc_inputs))
            self._plc_inputs = next_inputs

        plc_outputs = self._interface_items(payload.get("plc_outputs"))
        if plc_outputs is not None:
            self._plc_output_items = plc_outputs
            self._plc_outputs = dict(plc_outputs)

    def _cache_legacy_udp_message(self, message: str) -> None:
        self._last_plc_state = message.strip()
        self._last_plc_pending = None
        self._last_udp_display_message = message

    @staticmethod
    def _format_plc_status_display_message(
        state: str,
        pending: Optional[Dict[str, Any]],
    ) -> str:
        if pending is None:
            return state

        event = pending.get("event")
        source = pending.get("source")
        target = pending.get("target")
        steps = pending.get("steps")

        event = event if isinstance(event, str) else "UNKNOWN"
        source = source if isinstance(source, str) else "UNKNOWN"
        target = target if isinstance(target, str) else "UNKNOWN"
        steps_text = f" [{steps}]" if steps is not None else ""

        return f"{state} | pending {event}: {source} -> {target}{steps_text}"

    def getLastPlcStatusPayload(self) -> Dict[str, Any]:
        return dict(self._last_plc_status_payload)

    def getPlcState(self) -> Optional[str]:
        return self._last_plc_state

    def getPlcPending(self) -> Optional[Dict[str, Any]]:
        return dict(self._last_plc_pending) if self._last_plc_pending is not None else None

    def getLastUdpReceivedMonotonic(self) -> Optional[float]:
        return self._last_udp_received_monotonic

    def getLastUdpReceivedWallTime(self) -> Optional[str]:
        return self._last_udp_received_wall_time

    def getLastPlcStatusReceivedMonotonic(self) -> Optional[float]:
        return self._last_plc_status_received_monotonic

    def getLastPlcStatusReceivedWallTime(self) -> Optional[str]:
        return self._last_plc_status_received_wall_time

    def isUdpWatchdogExpired(self, timeout_s: float) -> bool:
        if self._last_udp_received_monotonic is None:
            return False
        return time.monotonic() - self._last_udp_received_monotonic > timeout_s

    def getUdpDisplayMessage(self) -> str:
        return self._last_udp_display_message

    def getPLCStates(self) -> Dict[str, Any]:
        return dict(self._plc_inputs)

    def getPLCOutputs(self) -> Dict[str, Any]:
        return dict(self._plc_outputs)

    def getPLCInputItems(self) -> List[Tuple[str, Any]]:
        return list(self._plc_input_items)

    def getPLCOutputItems(self) -> List[Tuple[str, Any]]:
        return list(self._plc_output_items)

    def isEmergencyActive(self) -> bool:
        if 'estop' not in self._plc_inputs:
            return True
        return not self._value_as_bool(self._plc_inputs['estop'])

    def isManualSwitchPressed(self) -> bool:
        return self._value_as_bool(self._plc_inputs.get('manual_switch_pressed', False))

    def getSlaveNames(self) -> List[str]:
        return list(self._cached_slave_names)

    def getSlaveStates(self) -> List[str]:
        return list(self._cached_slave_states)

    def _parse_plc_status(
        self,
        data: bytes,
    ) -> Tuple[str, Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:
        decoded = data.decode(errors="replace")
        try:
            payload = json.loads(decoded)
        except json.JSONDecodeError:
            return decoded, None, None

        if not isinstance(payload, dict):
            return decoded, None, None

        if payload.get("schema") != "fit4med.plc_fsm_status.v1":
            return decoded, None, None

        state = payload.get("state")
        pending = payload.get("pending")
        return (
            state if isinstance(state, str) else "",
            pending if isinstance(pending, dict) else None,
            payload,
        )

    def udpMessageReceived(self, data: bytes) -> None:
        """
        Handle UDP requests from the PLC manager. The PLC manager sends status updates
        to the GUI via UDP. The possible messages are:
            IDLE = auto()
            IDLE_RECOVERY = auto()
            ESTOP = auto()
            ESTOP_RECOVERY = auto()
            RUNNING = auto()
            RUNNING_RECOVERY = auto()
            RECOVERED = auto()
            ERROR = auto()
            ERROR_RECOVERY = auto()
        """
        state, pending, payload = self._parse_plc_status(data)
        if payload is not None:
            self._cache_plc_status_payload(payload, state, pending)
            self.plc_status_payload_received.emit(payload)
        else:
            self._cache_legacy_udp_message(state)
        pending_event = pending.get("event") if pending is not None else None

        if self._ros_reply == self.ROS_CONNECTION_FAILED and not self._is_plc_running():
            # The failure has been handled (the PLC left RUNNING). Keeping
            # FAILED would make plc_manager's START guard refuse the next start.
            print("[UdpServer] PLC left RUNNING after ROS failure: reporting ROS_DISCONNECTED.")
            self._ros_reply = self.ROS_DISCONNECTED

        #print(f"[UdpServer] {data.decode(errors='replace')} received from UDP client.")

        if pending_event in ("STOP", "FAIL"):
            self.requestRosCommunicationStop("UDP pending stop/fail transition received.")
            return

        if state in (
            'IDLE',
            'IDLE_RECOVERY',
            'ESTOP',
            'ESTOP_RECOVERY',
            'ERROR',
            'ERROR_RECOVERY',
            'RECOVERED',
        ) and pending is None:
            if not self.stop_ros_communication_emitted:
                self.requestRosCommunicationStop("Request from PLC via UDP to stop the ROS communication received.")

        elif state in self.PLC_RUNNING_STATES and pending is None:
            self.stop_ros_communication_emitted = False

            if self._ros_reply == self.ROS_CONNECTION_FAILED:
                # plc_manager answers the failure with FAIL: no reconnection
                # attempts until it has left RUNNING.
                return

            if (
                self.start_ros_communication_emitted
                and self._is_ros_communication_inactive()
            ):
                print("[UdpServer] RUNNING received but ROS communication is inactive. Retrying ROS start.")
                self.start_ros_communication_emitted = False

            if not self.start_ros_communication_emitted:
                self.start_ros_communication_emitted = True
                self.start_ros_communication.emit()

def main(args=None): #type: ignore
    # TODO
    pass

if __name__ == "__main__":
    main()
