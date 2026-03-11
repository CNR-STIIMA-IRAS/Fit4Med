# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from PyQt5.QtCore import QThread, QObject, pyqtSignal
from PyQt5.QtCore import QObject, pyqtSignal
import socket

################################################
#
#
#
################################################
class UdpServer(QObject):
    message_received = pyqtSignal(bytes, tuple)  # data, addr

    def __init__(self, host="0.0.0.0", port=5005, parent=None):
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

    def send_response(self, message: bytes, addr=None):
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

    def __init__(self, remote_ip, remote_port): #port=5005
        super().__init__()

        self.start_ros_communication_emitted = False
        self.stop_ros_communication_emitted = False
        self.remote_ip = remote_ip
        self.remote_port = remote_port
        self.roslib_first_time_connection = True
        self.udp_thread = QThread()
        self.server = UdpServer(port=remote_port)
        self.server.moveToThread(self.udp_thread)
        self.udp_thread.started.connect(self.server.start)
        
        self.server.message_received.connect(lambda d,a : self.udpRequestReceived(d))
        self.udp_thread.start()

    def onResetRosCommunication(self):
        self.server.send_response(b"STOPPED")
        self.start_ros_communication_emitted = False
        self.stop_ros_communication_emitted = False

    def udpRequestReceived(self, data):
        if data == b'STOP':
            print("[UdpServer] UDP STOP request received.")
            if not self.stop_ros_communication_emitted:
                self.stop_ros_communication.emit()
                self.stop_ros_communication_emitted = True
        elif data == b'RUNNING':
            if not self.start_ros_communication_emitted:
                self.start_ros_communication.emit()
                self.start_ros_communication_emitted = True
        else:
            print(f"[UdpServer] Unknown UDP packet received from UDP client: {data}")

def main(args=None):
    # TODO
    pass

if __name__ == "__main__":
    main()