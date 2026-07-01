# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

from contextlib import closing
import json
import socket

from plc_manager.plc_manager import UdpClient


def _status_payload(state: str, pending: dict | None = None) -> bytes:
    return json.dumps(
        {
            "schema": "fit4med.plc_fsm_status.v1",
            "state": state,
            "pending": pending,
        },
        separators=(",", ":"),
    ).encode()


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


def test_udp_client_sends_status_changes_immediately():
    """Semantic FSM changes should not wait for the periodic status send."""
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_sock.settimeout(1.0)
    with closing(server_sock):
        server_sock.bind(("127.0.0.1", 0))
        server_port = server_sock.getsockname()[1]

        client = UdpClient(
            target_ip="127.0.0.1",
            target_port=server_port,
            local_port=0,
            timeout=1.0,
            status_period=1.0,
        )

        idle = _status_payload("IDLE")
        pending_start = _status_payload(
            "IDLE",
            {
                "event": "START",
                "source": "IDLE",
                "target": "RUNNING",
                "steps": 1,
            },
        )
        running = _status_payload("RUNNING")

        try:
            client.send_status(idle)
            data, _ = server_sock.recvfrom(1024)
            assert data == idle

            client.send_status(pending_start)
            data, _ = server_sock.recvfrom(1024)
            assert data == pending_start

            client.send_status(running)
            data, _ = server_sock.recvfrom(1024)
            assert data == running
        finally:
            client.close()


def test_udp_client_decimates_repeated_status_and_resends_latest():
    """Repeated FSM state/pending keys are collapsed into periodic latest status."""
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_sock.settimeout(1.0)
    with closing(server_sock):
        server_sock.bind(("127.0.0.1", 0))
        server_port = server_sock.getsockname()[1]

        client = UdpClient(
            target_ip="127.0.0.1",
            target_port=server_port,
            local_port=0,
            timeout=1.0,
            status_period=0.3,
        )

        pending_step_1 = _status_payload(
            "IDLE",
            {
                "event": "START",
                "source": "IDLE",
                "target": "RUNNING",
                "steps": 1,
            },
        )
        pending_step_2 = _status_payload(
            "IDLE",
            {
                "event": "START",
                "source": "IDLE",
                "target": "RUNNING",
                "steps": 2,
            },
        )

        try:
            client.send_status(pending_step_1)
            data, _ = server_sock.recvfrom(1024)
            assert data == pending_step_1

            client.send_status(pending_step_2)
            server_sock.settimeout(0.1)
            try:
                server_sock.recvfrom(1024)
            except socket.timeout:
                pass
            else:
                raise AssertionError("Repeated status was sent before decimation period")

            server_sock.settimeout(1.0)
            data, _ = server_sock.recvfrom(1024)
            assert data == pending_step_2
        finally:
            client.close()
