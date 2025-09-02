from __future__ import print_function
import time
import roslibpy
import roslibpy.actionlib

client = roslibpy.Ros(host='10.2.16.42', port=9090)
client.run()
print('Is ROS connected?', client.is_connected)

listener = roslibpy.Topic(client, '/PLC_controller/plc_states', 'plc_controller_msgs/msg/PlcController')
listener.subscribe(lambda message: print(f'Heard talking: {message}'))


action_client = roslibpy.actionlib.ActionClient(client,
                                                '/joint_trajectory_controller/follow_joint_trajectory',
                                                'control_msgs/action/FollowJointTrajectory')

goal = {'trajectory': {'joint_names': ['joint_x', 'joint_y', 'joint_z'],
            'points': [{'positions': [0.0, 0.0, 0.0],
                        'velocities': [0.0, 0.0, 0.0],
                        'accelerations': [0.0, 0.0, 0.0],
                        'time_from_start': {'secs': 2, 'nsecs': 0}},
                       {'positions': [0.1, 0.1, 0.1],
                        'velocities': [0.0, 0.0, 0.0],
                        'accelerations': [0.0, 0.0, 0.0],
                        'time_from_start': {'secs': 4, 'nsecs': 0}},
                       {'positions': [0.2, 0.2, 0.2],
                        'velocities': [0.0, 0.0, 0.0],
                        'accelerations': [0.0, 0.0, 0.0],
                        'time_from_start': {'secs': 6, 'nsecs': 0}}]}}

goal = roslibpy.actionlib.Goal(action_client,
                               roslibpy.Message(goal))

goal.on('feedback', lambda f: print(f))
goal.send()
result = goal.wait(10)
action_client.dispose()

print(f'Result: {result}')
