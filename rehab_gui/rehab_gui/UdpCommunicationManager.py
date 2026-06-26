# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from PyQt5.QtCore import QThread, QObject, pyqtSignal, pyqtSlot
import socket
from typing import Any

################################################
#
#
#
################################################
class UdpServer(QObject):
    message_received = pyqtSignal(bytes, tuple)  # data, addr

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
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))
        self._running = True
        while self._running:
            try:
                data, addr = self.sock.recvfrom(4096)
                self._last_addr = addr
                self.message_received.emit(data, addr)
            except OSError as e:
                print(f"[UdpServer] OSError: {e}")
                break

    def stop(self):
        """Stop the server."""
        print("[UdpServer] Stopping server...")
        self._running = False
        if self.sock:
            self.sock.close()
        self.sock = None

    def send_response(self, message: bytes, addr: tuple[Any, ...] | None =None):
        """Send response to client."""
        if not self.sock:
            return
        if addr is None:
            addr = self._last_addr
        if addr:
            self.sock.sendto(message, addr)
            # print("[UdpServer] Sent response to {}: {}".format(addr, message))

################################################
#
#
#
################################################
class UdpCommunicationManager(QObject):

    start_ros_communication : pyqtSignal = pyqtSignal()
    stop_ros_communication : pyqtSignal = pyqtSignal()
    z_recovery_start_signal : pyqtSignal = pyqtSignal()
    z_recovery_mode_signal : pyqtSignal = pyqtSignal()
    z_recovery_done_signal : pyqtSignal = pyqtSignal()
    udp_message_received : pyqtSignal = pyqtSignal(bytes, tuple)

    def __init__(self, remote_ip: str, remote_port: int): #port=5005
        super().__init__()

        self.start_ros_communication_emitted  : bool = False
        self.stop_ros_communication_emitted  : bool = False
        self.z_recovery_start_signal_emitted : bool = False
        self.z_recovery_mode_signal_emitted : bool = False
        self.z_recovery_done_signal_emitted : bool = False

        self.remote_ip = remote_ip
        self.remote_port = remote_port
        self.roslib_first_time_connection = True
        self.udp_thread = QThread()
        self.server = UdpServer(port=remote_port)
        self.server.moveToThread(self.udp_thread)
        self.udp_thread.started.connect(self.server.start)

        self.server.message_received.connect(self.onUdpMessageReceived)
        self.udp_thread.start()

    @pyqtSlot(bytes, tuple)
    def onUdpMessageReceived(self, data: bytes, addr: str) -> None:
        self.udp_message_received.emit(data, addr)
        self.udpMessageReceived(data)

    def onResetRosCommunication(self) -> None:
        self.server.send_response(b"ROS_DISCONNECTED")
        self.start_ros_communication_emitted = False
        # self.stop_ros_communication_emitted = False

    @pyqtSlot()
    def onRosCommunicationEstablished(self) -> None:
        print("[UdpServer] ROS communication established.")
        self.server.send_response(b"ROS_CONNECTED")

    @pyqtSlot()
    def onRosCommunicationFailed(self) -> None:
        print("[UdpServer] ROS communication failed to start.")
        self.server.send_response(b"ROS_CONNECTION_FAILED")
        self.start_ros_communication_emitted = False

    def requestRosCommunicationStop(self, reason: str) -> None:
        print(f"[UdpServer] {reason}")
        if not self.stop_ros_communication_emitted:
            self.stop_ros_communication.emit()
            self.stop_ros_communication_emitted = True

    def udpMessageReceived(self, data: bytes) -> None:
        """
        Handle UDP requests from the PLC manager. The PLC manager sends status updates
        to the GUI via UDP. The possible messages are:
            IDLE = auto()
            IDLE_RECOVERY = auto()
            ESTOP = auto()
            RUNNING = auto()
            RUNNING_RECOVERY = auto()
            RECOVERED = auto()
            ERROR = auto()
        """
        print(f"[UdpServer] {data.decode()} received from UDP client.")
        if data in (b'IDLE', b'IDLE_RECOVERY', b'ERROR', b'STOP', b'RECOVERED'):
            self.requestRosCommunicationStop("UDP STOP request received.")
            if data == b'IDLE_RECOVERY' and not self.z_recovery_start_signal_emitted:
                self.z_recovery_start_signal.emit()
                self.z_recovery_start_signal_emitted = True
                self.z_recovery_done_signal_emitted = False
            elif data == b'RECOVERED' and not self.z_recovery_done_signal_emitted:
                self.z_recovery_done_signal.emit()
                self.z_recovery_done_signal_emitted = True
                self.z_recovery_mode_signal_emitted = False
                self.z_recovery_start_signal_emitted = False

        elif data == b'RUNNING' or data == b'RUNNING_RECOVERY':
            self.stop_ros_communication_emitted = False

            if not self.start_ros_communication_emitted:
                self.start_ros_communication.emit()
                self.start_ros_communication_emitted = True
                if data == b'RUNNING_RECOVERY' and not self.z_recovery_mode_signal_emitted:
                    self.z_recovery_mode_signal.emit()
                    self.z_recovery_mode_signal_emitted = True
                    self.z_recovery_done_signal_emitted = False
        else:
            print(f"[UdpServer] UDP packet received from UDP client: {data}")

def main(args=None): #type: ignore
    # TODO
    pass

if __name__ == "__main__":
    main()
