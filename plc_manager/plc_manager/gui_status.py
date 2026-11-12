# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

import json
from typing import Any


class GuiStatusPublisher:
    def __init__(self, client: Any, logger: Any) -> None:
        self.client = client
        self.logger = logger

    def fsm_status_payload(
        self,
        fsm: Any,
        ethercat_payload: dict[str, object],
        plc_outputs: Any,
        interface_names: list[str],
        state_values: list[int],
        announced_event: Any | None = None,
    ) -> bytes:
        """FSM status for the GUI.

        announced_event: a STOP/FAIL about to be triggered, sent as pending
        before its cleanup actions run (they stop rosbridge), so the GUI can
        close its ROS connection first. Ignored when a transition is pending
        or the FSM has no such transition from the current state.
        """
        pending = fsm.pending
        payload: dict[str, object] = {
            "schema": "fit4med.plc_fsm_status.v1",
            "state": fsm.state.name,
            "pending": None,
            "ethercat": ethercat_payload,
            "plc_outputs": {
                "interface_names": list(plc_outputs.interface_names),  # type: ignore
                "values": [int(value) for value in plc_outputs.values],  # type: ignore
            },
            "plc_inputs": {
                "interface_names": list(interface_names),
                "values": [int(value) for value in state_values],
            },
        }

        if pending is not None:
            payload["pending"] = {
                "event": pending.event.name,
                "source": pending.source.name,
                "target": pending.transition.destination.name,
                "steps": pending.steps,
            }
        elif announced_event is not None:
            transition = fsm.transitions.get((fsm.state, announced_event))
            if transition is not None:
                payload["pending"] = {
                    "event": announced_event.name,
                    "source": fsm.state.name,
                    "target": transition.destination.name,
                    "steps": 0,
                }

        return json.dumps(payload, separators=(",", ":")).encode()

    def notify(self, udp_msg: bytes) -> None:
        try:
            self.client.send_status(udp_msg)
        except Exception as exception:  # noqa: BLE001
            self.logger.warn(f"Error communicating with GUI: {exception}")  # type: ignore
