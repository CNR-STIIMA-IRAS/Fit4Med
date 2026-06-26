# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""ROS 2 service node that checks the EtherCAT PLC slave on demand."""

import os
import threading
import time
from typing import Optional

from ethercat_msgs.srv import GetSlaveStates
import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class EthercatSlavesStatusCheckNode(Node):
    """Offer an on-demand Trigger service for the EtherCAT PLC state check."""

    def __init__(self) -> None:
        super().__init__('ethercat_slaves_status_check')

        self.declare_parameter('plc_slave_identifier', 'sickPLC')
        self.declare_parameter('service_wait_timeout', 2.0)
        self.declare_parameter('response_timeout', 5.0)

        self._trigger_callback_group = MutuallyExclusiveCallbackGroup()
        self._client_callback_group = ReentrantCallbackGroup()

        self._check_service = self.create_service( #type: ignore
            Trigger,
            '~/check',
            self._check_callback,
            callback_group=self._trigger_callback_group,
        )

        self.get_logger().info( #type: ignore
            f'EtherCAT PLC status service ready: {self._check_service.srv_name}'
        )

    def _check_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Run one EtherCAT state check for a Trigger request."""
        response.success, response.message = self._check_plc_state()

        if response.success:
            self.get_logger().info(response.message) #type: ignore
        else:
            self.get_logger().warning(response.message) #type: ignore

        return response

    def _check_plc_state(self) -> tuple[bool, str]:
        """Query all EtherCAT state services and find the configured PLC."""
        if os.environ.get('PLC_MANAGER_SKIP_ETHERCAT', '0') == '1':
            return True, (
                'Skipped EtherCAT health check '
                '(PLC_MANAGER_SKIP_ETHERCAT=1).'
            )

        service_names = self._get_slave_state_service_names()
        if not service_names:
            return False, (
                'No EtherCAT GetSlaveStates services found on the ROS graph.'
            )

        service_wait_timeout = float(
            self.get_parameter('service_wait_timeout').value #type: ignore
        )
        response_timeout = float(
            self.get_parameter('response_timeout').value #type: ignore
        )
        plc_identifier = str(
            self.get_parameter('plc_slave_identifier').value #type: ignore
        )

        clients = []
        pending_requests = {}
        response_received = threading.Event()

        try:
            for service_name in service_names:
                client = self.create_client( #type: ignore
                    GetSlaveStates,
                    service_name,
                    callback_group=self._client_callback_group,
                )
                clients.append(client) #type: ignore

                if not client.wait_for_service(
                    timeout_sec=service_wait_timeout
                ):
                    continue

                future = client.call_async(GetSlaveStates.Request())
                future.add_done_callback( #type: ignore
                    lambda _future: response_received.set() #type: ignore
                )
                pending_requests[future] = service_name

            if not pending_requests:
                return False, (
                    'No EtherCAT GetSlaveStates service accepted a request.'
                )

            deadline = time.monotonic() + response_timeout
            observed_states: list[str] = []

            while pending_requests:
                remaining_time = deadline - time.monotonic()
                if remaining_time <= 0.0:
                    timed_out_services = ', '.join(
                        pending_requests.values() #type: ignore
                    )
                    return False, (
                        'Timed out waiting for EtherCAT state services: '
                        f'{timed_out_services}'
                    )

                response_received.wait(timeout=remaining_time)
                response_received.clear()

                for future, service_name in list( #type: ignore
                    pending_requests.items() #type: ignore
                ):
                    if not future.done(): #type: ignore
                        continue

                    del pending_requests[future]

                    try:
                        service_response = future.result() #type: ignore
                    except Exception as exception:  # noqa: BLE001
                        observed_states.append(
                            f'{service_name}: call failed ({exception})'
                        )
                        continue

                    for slave_name, slave_state in zip( #type: ignore
                        service_response.slave_names, #type: ignore
                        service_response.slave_states, #type: ignore
                    ):
                        if plc_identifier not in slave_name:
                            continue

                        if slave_state == 'OP':
                            return True, (
                                'EtherCAT PLC is operational: '
                                f'{slave_name} => {slave_state} '
                                f'via {service_name}'
                            )

                        observed_states.append(
                            f'{slave_name} => {slave_state} '
                            f'via {service_name}'
                        )

            if observed_states:
                return False, (
                    'EtherCAT PLC is not operational: '
                    + '; '.join(observed_states)
                )

            return False, (
                f"EtherCAT PLC slave '{plc_identifier}' was not found."
            )
        finally:
            for client in clients: #type: ignore
                self.destroy_client(client) #type: ignore

    def _get_slave_state_service_names(self) -> list[str]:
        """Discover EtherCAT GetSlaveStates services for this request."""
        return sorted(
            name
            for name, service_types in self.get_service_names_and_types()
            if 'ethercat_msgs/srv/GetSlaveStates' in service_types
        )


def main(args: Optional[list[str]] = None) -> None:
    """Run the service node until ROS shuts down."""
    rclpy.init(args=args)
    node = EthercatSlavesStatusCheckNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown() #type: ignore


if __name__ == '__main__':
    main()
