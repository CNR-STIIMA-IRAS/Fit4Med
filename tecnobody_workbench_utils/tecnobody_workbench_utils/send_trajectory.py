import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Int16

JOINTS = [
    'joint_1',
    'joint_2',
    'joint_3'
]

class TrajectoryActionClient(Node):

    def __init__(self):
        super().__init__('tecnobody_workbench_utils') # type: ignore
        self._action_client = ActionClient(self, FollowJointTrajectory, '/ft_sensor/joint_trajectory_controller/follow_joint_trajectory')
        # self._publisher = self.create_publisher(Int16, '/speed_ovr', 10)
        self.get_logger().info("Trajectory client node started.")

    def load_parameters(self):
        param_file_path = os.path.join(
            get_package_share_directory('tecnobody_workbench_utils'),
            'config',
            'trajectory_params.yaml'
        )

        # Load the YAML file
        with open(param_file_path, 'r') as file:
            params = yaml.safe_load(file)

        # Extract cart_coordinates and duration_list from parameters
        self._cart_coordinates = params['cart_coordinates']
        self._velocities = params['velocities']
        self._duration_list = params['durations']

        self.get_logger().info("Goal correctly loaded")


    def send_goal(self, names):
        server_available = self._action_client.wait_for_server()
        self.get_logger().info("Sending goal")

        if server_available:
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = names
            
            for p in range(0, len(self._cart_coordinates)):
                point = JointTrajectoryPoint()
                point.positions = self._cart_coordinates[p]
                point.velocities = [] # self._velocities[p]
                point.accelerations = []
                point.effort = []
                point.time_from_start = Duration(sec=int(self._duration_list[p]), nanosec=int((self._duration_list[p] - int(self._duration_list[p])) * 1e9))
                self.get_logger().info(f"Goal Message: {goal_msg}")
                goal_msg.trajectory.points.append(point)
        else:
           self.get_logger().error("Action server not available!!")
           return -1

        if len(self._cart_coordinates) != len(self._duration_list):
            self.get_logger().error("Different positions and times lenghts in configuration file!!")
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
