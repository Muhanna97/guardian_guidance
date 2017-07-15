import json

def getInteropJSON():

    with open('surrey_mission.json') as data_file_one:  # chage file name to missions for use
        mission_data = json.load(data_file_one)

        waypoints = mission_data[u'mission_waypoints'] # pull this list out of the dict

        lats = []
        longs = []
        alts = []

        for item in waypoints:
            lats.append(item[u'latitude'])
            longs.append(item[u'longitude'])
            alts.append(item[u'altitude_msl'])

    # reformat the json into a non-gross dict. Only supporting one obstacle at the moment
    wpout = {'wp_lats': lats,
             'wp_longs': longs,
             'wp_alts': alts,
             'drop_lat': mission_data[u'air_drop_pos'][u'latitude'],
             'drop_long': mission_data[u'air_drop_pos'][u'longitude'],
             'obstacle_lat': mission_data[u'stationary_obstacles'][0][u'latitude'],
             'obstacle_long':  mission_data[u'stationary_obstacles'][0][u'longitude'],
             }
    # print wpout




# copy of an import json for dev purposes
        # {u'stationary_obstacles': [
        #     {u'latitude': 38.14792, u'cylinder_height': 200.0, u'cylinder_radius': 150.0, u'longitude': -76.427995},
        #     {u'latitude': 38.145823, u'cylinder_height': 300.0, u'cylinder_radius': 100.0, u'longitude': -76.422396}],
        #  u'fly_zones': [{u'boundary_pts': [{u'latitude': 38.142544, u'order': 1, u'longitude': -76.434088},
        #                                    {u'latitude': 38.141833, u'order': 2, u'longitude': -76.425263},
        #                                    {u'latitude': 38.144678, u'order': 3, u'longitude': -76.427995}],
        #                  u'altitude_msl_max': 750.0, u'altitude_msl_min': 0.0}],
        #  u'off_axis_target_pos': {u'latitude': 38.142544, u'longitude': -76.434088},
        #  u'moving_obstacles': [{u'sphere_radius': 50.0, u'speed_avg': 30.0}],
        #  u'mission_waypoints': [{u'latitude': 38.142544, u'altitude_msl': 200.0, u'order': 1, u'longitude': -76.434088},
        #                         {u'latitude': 38.143544, u'altitude_msl': 200.0, u'order': 2, u'longitude': -76.435088},
        #                         {u'latitude': 38.148544, u'altitude_msl': 200.0, u'order': 3,
        #                          u'longitude': -76.434388}],
        #  u'emergent_last_known_pos': {u'latitude': 38.145823, u'longitude': -76.422396}, u'search_grid_points': [
        #     {u'latitude': 38.142544, u'altitude_msl': 200.0, u'order': 1, u'longitude': -76.434088}], u'active': True,
        #  u'id': 1, u'home_pos': {u'latitude': 38.14792, u'longitude': -76.427995},
        #  u'air_drop_pos': {u'latitude': 38.141833, u'longitude': -76.425263}}


    return wpout

