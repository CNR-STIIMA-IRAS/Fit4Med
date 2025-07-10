import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

class FTOffsetUpdater(Node):
    def __init__(self):
        super().__init__('ft_offset_updater')

        # Sottoscrizione al topic del sensore di forza-torque
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/ft_sensor_command_broadcaster/wrench',
            self.wrench_callback,
            10
        )

        # Create wrench offset topic
        self.ft_offset_pub = self.create_publisher(WrenchStamped, '/admittance_controller/wrench_reference', 10)
        
        # Timer to stop the node after 10 seconds
        self.bias_timer = self.create_timer(10.0, self.compute_bias)
        self.get_logger().info('FTOffsetUpdater initialized and listening wrench topic.')

        # Initialize variables
        self.bias = []
        self.deadband = 3.0
        self.ft_queue = []
        self.ft_measures = np.zeros(6)
        self.filtered_ft_ref = []
        self._bias_computed = False


    def wrench_callback(self, msg):
        force = msg.wrench.force
        torque = msg.wrench.torque
        offset_force = np.zeros(6)
        self.ft_measures[0] = force.x
        self.ft_measures[1] = force.y
        self.ft_measures[2] = force.z
        self.ft_measures[3] = torque.x
        self.ft_measures[4] = torque.y
        self.ft_measures[5] = torque.z

        if not self._bias_computed:
            self.ft_queue.append(self.ft_measures)
            offset_force = -self.ft_measures
        else: 
            
            for idx in range(6):
                _m = self.ft_measures[idx]-self.bias[idx]
                if _m > self.deadband:
                    offset_force[idx] = - self.bias[idx] - self.deadband
                elif _m < -self.deadband:
                    offset_force[idx] = - self.bias[idx] + self.deadband
                else: 
                    offset_force[idx] = -self.ft_measures[idx]
    
        offset_force_msg = WrenchStamped()
        offset_force_msg.header.stamp = self.get_clock().now().to_msg()
        offset_force_msg.header.frame_id = "ee_link"
        offset_force_msg.wrench.force.x = offset_force[0]
        offset_force_msg.wrench.force.y = offset_force[1]
        offset_force_msg.wrench.force.z = offset_force[2]
        offset_force_msg.wrench.torque.x = offset_force[3]
        offset_force_msg.wrench.torque.y = offset_force[4]
        offset_force_msg.wrench.torque.z = offset_force[5]
        self.ft_offset_pub.publish(offset_force_msg)


    def compute_bias(self):
        self.bias = np.mean(np.asarray(self.ft_queue),0)
        self._bias_computed = True
        self.bias_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = FTOffsetUpdater()
    try:
        rclpy.spin(node)
    except:
        node.get_logger().info('Shutting down.\n')

if __name__ == '__main__':
    main()
