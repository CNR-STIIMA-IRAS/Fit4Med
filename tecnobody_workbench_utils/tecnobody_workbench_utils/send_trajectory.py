import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINTS = [
    'joint_x',
    'joint_y',
    'joint_z'
]

class TrajectoryActionClient(Node):

    def __init__(self):
        super().__init__('trajectory_action_client')
        
        # Declare default params
        self.declare_parameter('controller_name', 'joint_trajectory_controller')
        controller_name = self.get_parameter('controller_name').value

        # Compose action server name
        action_server_name = f'/{controller_name}/follow_joint_trajectory'
        self.get_logger().info(f"Using action server: {action_server_name}")

        # Create action client
        self._action_client = ActionClient(self, FollowJointTrajectory, action_server_name)

        self.get_logger().info("Trajectory client node started.")

    def load_parameters(self):
        param_file_path = os.path.join(
            get_package_share_directory('tecnobody_workbench_utils'),
            'config',
            'trajectory_params.yaml'
        )

        with open(param_file_path, 'r') as file:
            params = yaml.safe_load(file)

        self._cart_coordinates = params['cart_coordinates']
        self._velocities = params['velocities']
        self._accelerations = params['accelerations']
        self._duration_list = params['durations']

        self.get_logger().info("Goal correctly loaded")

    def send_goal(self, names):
        server_available = self._action_client.wait_for_server()
        self.get_logger().info("Sending goal")

        if server_available:
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = names

            for p in range(len(self._cart_coordinates)):
                point = JointTrajectoryPoint()
                point.positions = self._cart_coordinates[p]
                point.velocities = self._velocities[p]
                point.accelerations = self._accelerations[p]
                point.time_from_start = Duration(
                    sec=int(self._duration_list[p]),
                    nanosec=int((self._duration_list[p] - int(self._duration_list[p])) * 1e9)
                )
                goal_msg.trajectory.points.append(point)
        else:
            self.get_logger().error("Action server not available!!")
            return -1

        if len(self._cart_coordinates) != len(self._duration_list):
            self.get_logger().error("Different positions and times lengths in configuration file!!")
            return -1
            
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    action_client = TrajectoryActionClient()
    action_client.load_parameters()

    goal_sent = action_client.send_goal(JOINTS)
    rclpy.spin_until_future_complete(action_client, goal_sent)

if __name__ == '__main__':
    main()
