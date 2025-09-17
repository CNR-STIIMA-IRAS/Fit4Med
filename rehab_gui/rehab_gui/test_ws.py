from __future__ import print_function
import time
import roslibpy
import numpy as np

# import roslibpy.actionlib

def get_op_mode_number(mode):
    op_mode_dict = {
        'MODE_NO_MODE': 0,
        'MODE_PROFILED_POSITION': 1,
        'MODE_PROFILED_VELOCITY': 3,
        'MODE_PROFILED_TORQUE': 4,
        'MODE_HOMING': 6,
        'MODE_INTERPOLATED_POSITION': 7,
        'MODE_CYCLIC_SYNC_POSITION': 8,
        'MODE_CYCLIC_SYNC_VELOCITY': 9,
        'MODE_CYCLIC_SYNC_TORQUE': 10
    }
    return op_mode_dict.get(mode, 0)

client = roslibpy.Ros(host='10.2.15.249', port=9090)
client.run()
# print('Is ROS connected?', client.is_connected)

# TOPIC LISTENER EXAMPLE
#listener = roslibpy.Topic(client, '/PLC_controller/plc_states', 'tecnobody_msgs/msg/PlcController')
#listener.subscribe(lambda message: print(f'Heard talking: {message}'))

# SERVICE EXAMPLE

get_op_mode_client = roslibpy.Service(client, '/ethercat_checker/get_drive_mode_of_operation',
                                           'ethercat_controller_msgs/srv/GetDriveModeOfOperation')

JOINT_NAMES = [
    'joint_x',
    'joint_y',
    'joint_z'
]
req = roslibpy.ServiceRequest({
            'dof_names': JOINT_NAMES
        })
print('>>>>>>>>>>>>>>>>>>>>>>> Calling service...')
result = get_op_mode_client.call(req)
print(f'Service response: {result}')
print(f'Service response: {result["values"]}')
results_int = []
for mode in result["values"]:
    results_int.append(get_op_mode_number(mode))
print(f'Service response: {results_int}')

client.close()

# del client
time.sleep(1)

#SECOND TIME--------------------------------------------------------------------
# client = roslibpy.Ros(host='10.2.15.249', port=9090)
client.connect()
service = roslibpy.Service(client, '/controller_manager/list_controllers',
                                                          'controller_manager_msgs/srv/ListControllers')
request = roslibpy.ServiceRequest()
print('>>>>>>>>>>>>>>>>>>>>>>>>> Calling service...')
result = service.call(request)
print(f'Service response:')
for ctrl in result['controller']:
    print(f"Controller: {ctrl['name']} - Type: {ctrl['type']} - State: {ctrl['state']}")

# ACTION CLIENT EXAMPLE
# print(f'Creating the action client')
# action_client = roslibpy.actionlib.ActionClient(client,
#                                                 '/joint_trajectory_controller/follow_joint_trajectory',
#                                                 'control_msgs/action/FollowJointTrajectory')
#
# print(f'Creating the goal')
# goal = {'trajectory': {'joint_names': ['joint_x', 'joint_y', 'joint_z'],
#             'points': [{'positions': [0.0, 0.0, 0.0],
#                         'velocities': [0.0, 0.0, 0.0],
#                         'accelerations': [0.0, 0.0, 0.0],
#                         'time_from_start': {'secs': 2, 'nsecs': 0}},
#                        {'positions': [0.1, 0.1, 0.1],
#                         'velocities': [0.0, 0.0, 0.0],
#                         'accelerations': [0.0, 0.0, 0.0],
#                         'time_from_start': {'secs': 4, 'nsecs': 0}},
#                        {'positions': [0.2, 0.2, 0.2],
#                         'velocities': [0.0, 0.0, 0.0],
#                         'accelerations': [0.0, 0.0, 0.0],
#                         'time_from_start': {'secs': 6, 'nsecs': 0}}]}}
#
# print(f'Sending goal: {goal}')
# goal = roslibpy.actionlib.Goal(action_client,
#                                roslibpy.Message(goal))
# print(f'Sending goal: {roslibpy.Message(goal)}')
#
# #goal.on('feedback', lambda f: print(f))
# goal.send()
# result = goal.wait(10)
# action_client.dispose()
# print(f'Result: {result}')

client.terminate()

