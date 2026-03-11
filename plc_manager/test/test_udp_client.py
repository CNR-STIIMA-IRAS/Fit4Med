# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

from contextlib import closing
import socket

from plc_manager.plc_manager import UdpClient


def test_udp_client_round_trip():
    """Verify that UdpClient can send and receive a short datagram."""
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_sock.settimeout(1.0)
    with closing(server_sock):
        server_sock.bind(("127.0.0.1", 0))
        server_port = server_sock.getsockname()[1]

        client = UdpClient(
            target_ip="127.0.0.1", target_port=server_port, local_port=0, timeout=1.0
        )
        assert client is not None

        try:
            client.send(b"TEST")
            data, addr = server_sock.recvfrom(1024)
            assert data == b"TEST"

            server_sock.sendto(b"ACK", addr)
            response, _ = client.receive()
            assert response == b"ACK"
        finally:
            client.close()
