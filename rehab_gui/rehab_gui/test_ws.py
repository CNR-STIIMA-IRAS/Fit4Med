import roslibpy

client = roslibpy.Ros(host='10.2.16.42', port=9090)
client.run()
print('Is ROS connected?', client.is_connected)
client.terminate()