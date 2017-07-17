#! /usr/bin/python
import sys
import os
import inspect

cmd_subfolder = os.path.realpath(
    os.path.abspath(os.path.join(os.path.split(inspect.getfile(inspect.currentframe()))[0], "../interop/client/")))
if cmd_subfolder not in sys.path:
    sys.path.insert(0, cmd_subfolder)

from interop import client, types

if __name__ == '__main__':

    server = "http://192.168.0.197:8000"
    username = 'testuser'
    password = 'testpass'

    interop_client = client.Client(server, username, password, timeout=2, max_retries=1)

    missions = interop_client.get_missions()
    mission = missions[0]

    obstacles = interop_client.get_obstacles()
    stationary_obstacles = obstacles[0]
    moving_obstacles = obstacles[1]
