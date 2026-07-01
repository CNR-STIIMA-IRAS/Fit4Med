# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
"""ROS 2 node that exposes /bag_recorder/start and /bag_recorder/stop services.

The services execute the bash_scripts/start_bag.sh and stop_bag.sh scripts
that live in the same Fit4Med source tree.  All ros2 sourcing, directory
creation and bag management is delegated to the shell scripts so this node
stays minimal and platform-agnostic.
"""

import os
import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger


class BagRecorderNode(Node):

    def __init__(self) -> None:
        super().__init__('bag_recorder_node')

        # Default path works whether colcon uses --symlink-install or not.
        # The bash_scripts/ dir lives at a fixed location in the source tree.
        default_scripts_dir = os.path.expanduser(
            '~/fit4med_ws/src/Fit4Med/bash_scripts'
        )

        self.declare_parameter('scripts_dir', default_scripts_dir)
        self._scripts_dir: str = self.get_parameter('scripts_dir').value

        self._start_srv = self.create_service(
            Trigger, '/bag_recorder/start', self._handle_start)
        self._stop_srv = self.create_service(
            Trigger, '/bag_recorder/stop', self._handle_stop)

        self.get_logger().info(
            f'BagRecorderNode ready. Scripts dir: {self._scripts_dir}')

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _handle_start(
            self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        script = os.path.join(self._scripts_dir, 'start_bag.sh')
        if not os.path.isfile(script):
            msg = f'start_bag.sh not found at {script}'
            self.get_logger().error(msg)
            response.success = False
            response.message = msg
            return response

        try:
            # Non-blocking: start_bag.sh runs in the background and blocks
            # internally on "wait $BAG_PID" until stop_bag.sh kills it.
            subprocess.Popen(['bash', script])
            response.success = True
            response.message = 'Bag recording started.'
            self.get_logger().info('Bag recording started.')
        except Exception as exc:
            msg = f'Failed to start bag recording: {exc}'
            self.get_logger().error(msg)
            response.success = False
            response.message = msg

        return response

    def _handle_stop(
            self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        script = os.path.join(self._scripts_dir, 'stop_bag.sh')
        if not os.path.isfile(script):
            msg = f'stop_bag.sh not found at {script}'
            self.get_logger().error(msg)
            response.success = False
            response.message = msg
            return response

        try:
            result = subprocess.run(['bash', script], timeout=10, check=False)
            response.success = result.returncode == 0
            response.message = 'Bag recording stopped.'
            self.get_logger().info('Bag recording stopped.')
        except Exception as exc:
            msg = f'Failed to stop bag recording: {exc}'
            self.get_logger().error(msg)
            response.success = False
            response.message = msg

        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BagRecorderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, SystemExit):
            pass
        except Exception:
            pass

        try:
            rclpy.try_shutdown()
        except (KeyboardInterrupt, SystemExit):
            pass


if __name__ == '__main__':
    main()
