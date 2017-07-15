#! /usr/bin/python
import sys
import os
import inspect


def importFromInterop():
    cmd_subfolder = os.path.realpath(
        os.path.abspath(os.path.join(os.path.split(inspect.getfile(inspect.currentframe()))[0], "../interop/client/")))
    if cmd_subfolder not in sys.path:
        sys.path.insert(0, cmd_subfolder)

    from interop import client, types

    # if __name__ == '__main__':

    server = "http://192.168.0.197:8000"
    username = 'testuser'
    password = 'testpass'

    interop_client = client.Client(server, username, password, timeout=2, max_retries=1)

    missions = interop_client.get_missions()
    mission = missions[0]

    obstacles = interop_client.get_obstacles()
    stationary_obstacles = obstacles[0]
    moving_obstacles = obstacles[1]

    guardianWp = []

    print stationary_obstacles

    # use the waypoints and search grid points to generate waypoints to fly
    for Waypoint in mission.mission_waypoints:
        guardianWp.append(Waypoint)
    for Waypoint in mission.search_grid_points:
        guardianWp.append(Waypoint)

    guardianWp.sort(key=lambda x: x.order, reverse=False) # sort low>high

# construct dict to sent to guidance

    lats = []
    longs = []
    alts = []

    for Waypoint in guardianWp:
        lats.append(Waypoint.latitude)
        longs.append(Waypoint.longitude)
        alts.append(Waypoint.altitude_msl)

    wpout = {'wp_lats': lats,
             'wp_longs': longs,
             'wp_alts': alts,
             'drop_lat': mission.air_drop_pos.latitude,
             'drop_long': mission.air_drop_pos.longitude,
             'obstacle_lat': stationary_obstacles[0].latitude,
             'obstacle_long':  stationary_obstacles[0].longitude,
             'obstacle_radius': stationary_obstacles.cylinder_radius,
             }

    return wpout


