from PyQt5.QtCore import QObject, pyqtSignal
import socket

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
        """Avvia il server UDP (bloccante, da lanciare in un QThread)."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))
        self._running = True
        while self._running:
            try:
                data, addr = self.sock.recvfrom(4096)
                self._last_addr = addr
                self.message_received.emit(data, addr)
            except OSError:
                break

    def stop(self):
        """Ferma il server."""
        print("[UdpServer] Stopping server...")
        self._running = False
        if self.sock:
            self.sock.close()
        self.sock = None

    def send_response(self, message: bytes, addr=None):
        """Risponde al client (se addr è None usa l'ultimo)."""
        if not self.sock:
            return
        if addr is None:
            addr = self._last_addr
        if addr:
            self.sock.sendto(message, addr)
            # print("[UdpServer] Sent response to {}: {}".format(addr, message))
