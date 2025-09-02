from __future__ import print_function
import time
import roslibpy

client = roslibpy.Ros(host='10.2.16.42', port=9090)
client.run()
print('Is ROS connected?', client.is_connected)
client.terminate()

listener = roslibpy.Topic(client, '/joint_states', 'sensor_msgs/msg/JointState')
listener.subscribe(lambda message: print('Heard talking: ' + message['data']))

try:
    while True:
        pass
except KeyboardInterrupt:
    client.terminate()
client.terminate()
