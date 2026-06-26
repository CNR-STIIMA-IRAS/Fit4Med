#!/usr/bin/env python3
import socket
import threading
import time
from typing import Any, Callable


class UdpClient:
    """UDP client with asynchronous receive support.

    The socket receive path is owned by one background thread. Callers can send
    freely, inspect the latest received packet, or optionally wait for a
    specific packet without calling recvfrom() from multiple places.
    """

    def __init__(
        self,
        target_ip: str = "127.0.0.1",
        target_port: int = 5005,
        local_port: int = 0,
        timeout: float = 2.0,
        on_message: Callable[[bytes, Any], None] | None = None,
    ):
        self.target_ip = target_ip
        self.target_port = target_port
        self.timeout = timeout
        self._on_message = on_message

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", local_port))

        # Short timeout so the receive thread can stop quickly.
        self.sock.settimeout(0.2)

        self._stop_event = threading.Event()
        self._condition = threading.Condition()
        self._messages: list[tuple[bytes, Any]] = []
        self._last_message: tuple[bytes, Any] | None = None
        self._max_messages = 100

        self._rx_thread = threading.Thread(
            target=self._receive_loop,
            name="UdpClientRxThread",
            daemon=True,
        )
        self._rx_thread.start()

    def send(self, message: bytes) -> None:
        self.sock.sendto(message, (self.target_ip, self.target_port))

    def _receive_loop(self, bufsize: int = 4096) -> None:
        while not self._stop_event.is_set():
            try:
                data, addr = self.sock.recvfrom(bufsize)
            except socket.timeout:
                continue
            except OSError:
                break

            print(f"[UdpClient] Received message from {addr}: {data}")

            with self._condition:
                self._last_message = (data, addr)
                self._messages.append((data, addr))
                if len(self._messages) > self._max_messages:
                    self._messages.pop(0)
                self._condition.notify_all()

            if self._on_message is not None:
                try:
                    self._on_message(data, addr)
                except Exception as exc:
                    print(f"[UdpClient] Error in on_message callback: {exc}")

    @property
    def last_received_message(self) -> bytes | None:
        with self._condition:
            if self._last_message is None:
                return None
            return self._last_message[0]

    @property
    def last_sender_address(self) -> Any | None:
        with self._condition:
            if self._last_message is None:
                return None
            return self._last_message[1]

    def get_last_message(self) -> tuple[bytes | None, Any | None]:
        with self._condition:
            if self._last_message is None:
                return None, None
            return self._last_message

    def clear_last_message(self) -> None:
        with self._condition:
            self._last_message = None

    def receive(self, timeout: float | None = None) -> tuple[bytes | None, Any | None]:
        wait_time = self.timeout if timeout is None else timeout
        deadline = time.monotonic() + wait_time

        with self._condition:
            while not self._messages:
                if self._stop_event.is_set():
                    return None, None
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None, None
                self._condition.wait(remaining)

            return self._messages.pop(0)

    def wait_for_message(
        self,
        expected: bytes,
        timeout: float | None = None,
    ) -> tuple[bytes | None, Any | None]:
        wait_time = self.timeout if timeout is None else timeout
        deadline = time.monotonic() + wait_time

        with self._condition:
            while True:
                for idx, (data, addr) in enumerate(self._messages):
                    if data == expected:
                        del self._messages[idx]
                        return data, addr

                remaining = deadline - time.monotonic()
                if remaining <= 0 or self._stop_event.is_set():
                    return None, None

                self._condition.wait(remaining)

    def close(self) -> None:
        if self._stop_event.is_set():
            return

        self._stop_event.set()
        with self._condition:
            self._condition.notify_all()

        try:
            self.sock.close()
        except OSError:
            pass

        self._rx_thread.join(timeout=1.0)
