import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile
import time
import threading
import numpy as np


class FakeFollowJointTrajectoryServer(Node):

    def __init__(self):
        super().__init__('fake_follow_joint_trajectory_server')

        # Publisher /joint_states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', QoSProfile(depth=10))

        # Init joint state
        self.joint_names = ["joint_1", "joint_2", "joint_3"]  # Default joint names
        self.current_positions = [0.0, 0.0, 0.0]  # Default positions
        self.lock = threading.Lock()

        # Start action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/fmrrehab_controller/follow_joint_trajectory',
            self.execute_callback
        )

        # Start a background thread to publish joint_states
        self.keep_publishing = True
        self.publisher_thread = threading.Thread(target=self.publish_joint_states_loop)
        self.publisher_thread.start()

        self.get_logger().info('Fake FollowJointTrajectory server with joint_states started.')

    def publish_joint_states_loop(self):
        rate = self.create_rate(50)  # 50 Hz
        msg = JointState()

        while rclpy.ok() and self.keep_publishing:
            with self.lock:
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = self.joint_names
                msg.position = self.current_positions.copy()
            self.joint_pub.publish(msg)
            rate.sleep()

    def execute_callback(self, goal_handle):
        traj = goal_handle.request.trajectory
        self.get_logger().info(f'Received trajectory with {len(traj.points)} points.')

        # Init joint names and positions
        with self.lock:
            if not self.joint_names or self.joint_names == ["joint_1", "joint_2", "joint_3"]:
                self.joint_names = traj.joint_names
                self.current_positions = [0.0] * len(self.joint_names)

        start_time = self.get_clock().now()

        for i, point in enumerate(traj.points):
            # Time from start
            t_target = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            t_now = (self.get_clock().now() - start_time).nanoseconds * 1e-9

            duration = max(t_target - t_now, 0.01)
            steps = int(duration * 50)  # 50 Hz

            with self.lock:
                start_pos = np.array(self.current_positions)
                end_pos = np.array(point.positions)

            for step in range(steps):
                alpha = (step + 1) / steps
                new_pos = (1 - alpha) * start_pos + alpha * end_pos
                with self.lock:
                    self.current_positions = new_pos.tolist()
                time.sleep(1.0 / 50)

            # Feedback
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = self.joint_names
            feedback.desired = point
            feedback.actual = JointTrajectoryPoint(positions=self.current_positions)
            feedback.error = JointTrajectoryPoint()
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = 0
        self.get_logger().info('Fake goal execution completed.')
        return result

    def destroy_node(self):
        self.keep_publishing = False
        self.publisher_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FakeFollowJointTrajectoryServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down on SIGINT.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
