import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class FTOffsetUpdater(Node):
    def __init__(self):
        super().__init__('ft_offset_updater')

        # Param Updater Node
        self.target_node = '/ft_sensor_command_broadcaster'
        self.verbose_mode = True

        # Create client for setting params of the target node (ft broadcaster)
        self.param_client = self.create_client(SetParameters, f'{self.target_node}/set_parameters')
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for the service: {self.target_node}/set_parameters...')

        # Sottoscrizione al topic del sensore di forza-torque
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/ft_sensor_command_broadcaster/wrench',
            self.wrench_callback,
            10
        )
        
        # Timer to stop the node after 10 seconds
        self.timer = self.create_timer(10.0, self.shutdown_sub)
        self.get_logger().info('FTOffsetUpdater initialized and listening wrench topic.')

        # Initialize offset list
        self.off_x = []
        self.off_y = []
        self.off_z = []
        self.off_tx = []
        self.off_ty = []
        self.off_tz = []

        # Flag to check if the node is shutting down
        self.shutting_down = False

    def create_parameter(self, name, value):
        return Parameter(
            name=name,
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
        )

    def wrench_callback(self, msg):
        if self.shutting_down:
            return

        force = msg.wrench.force
        torque = msg.wrench.torque

        self.off_x.append(force.x)
        self.off_y.append(force.y)
        self.off_z.append(force.z)
        self.off_tx.append(torque.x)
        self.off_ty.append(torque.y)
        self.off_tz.append(torque.z)

    def update_parameters(self):
        if not self.shutting_down:
            return
        
        offset_x = np.mean(self.off_x).astype(np.double)
        offset_y = np.mean(self.off_y).astype(np.double)
        offset_z = np.mean(self.off_z).astype(np.double)
        offset_tx = np.mean(self.off_tx).astype(np.double)
        offset_ty = np.mean(self.off_ty).astype(np.double)
        offset_tz = np.mean(self.off_tz).astype(np.double)

        if self.verbose_mode:
            print('Updating offset parameters...\n')
            print(f'Offset x: {offset_x}')
            print(f'Offset y: {offset_y}')
            print(f'Offset z: {offset_z}')
            print(f'Offset tx: {offset_tx}')
            print(f'Offset ty: {offset_ty}')
            print(f'Offset tz: {offset_tz}')
            print('\n')

        # Create params to update
        params = [
            self.create_parameter('offset.force.x', -offset_x),
            self.create_parameter('offset.force.y', -offset_y),
            self.create_parameter('offset.force.z', -offset_z),
            self.create_parameter('offset.torque.x', -offset_tx),
            self.create_parameter('offset.torque.y', -offset_ty),
            self.create_parameter('offset.torque.z', -offset_tz),
        ]

        # Call param update service
        request = SetParameters.Request(parameters=params)
        future = self.param_client.call_async(request)
        future.add_done_callback(self.on_param_update_response)

    def on_param_update_response(self, future):
        try:
            response = future.result()
            if all(result.successful for result in response.results):
                self.get_logger().info('Parameters updated successfully.')
                self.shutdown_node()
            else:
                self.get_logger().warn("Error updating one or more parameters.")
        except Exception as e:
            if not self.shutting_down:
                self.get_logger().error(f'Error when calling the service: {e}')

    def shutdown_sub(self):
        self.shutting_down = True
        if self.destroy_subscription(self.subscription):
            self.get_logger().info('Biasing completed. Unsubscribing from wrench topic.')
            self.update_parameters()

    def shutdown_node(self):
        self.get_logger().info('Shutting down FTOffsetUpdater...')
        self.timer.cancel()
        self.destroy_client(self.param_client)
        self.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FTOffsetUpdater()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
