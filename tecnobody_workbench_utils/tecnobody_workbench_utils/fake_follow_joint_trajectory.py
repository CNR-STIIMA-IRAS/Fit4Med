import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.qos import QoSProfile
import time

class FakeFollowJointTrajectoryServer(Node):

    def __init__(self):
        super().__init__('fake_follow_joint_trajectory_server')
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/fmrrehab_controller/follow_joint_trajectory',
            self.execute_callback
        )
        self.get_logger().info('Fake FollowJointTrajectory server started!')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal with {len(goal_handle.request.trajectory.points)} points.')

        # Finge di eseguire il goal inviando feedback
        for i, point in enumerate(goal_handle.request.trajectory.points):
            time.sleep(0.1)  # Simula il tempo di esecuzione
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = goal_handle.request.trajectory.joint_names
            feedback.desired = point
            feedback.actual = JointTrajectoryPoint()
            feedback.error = JointTrajectoryPoint()
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = 0  # SUCCESSFUL
        self.get_logger().info('Goal executed successfully (fake).')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FakeFollowJointTrajectoryServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user (SIGINT).')
    finally:
        node.destroy_node()
        rclpy.shutdown()
