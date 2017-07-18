from math import cos, sin, pi, radians, degrees


def findNextLatLng(old_lat, old_lng, d_north, d_east, emergent_object_lkp_lat, emergent_object_lkp_lng):
    R = 6378137.0 # spherical earth radius (decimal to ensure we use floats)

    # Coordinate offsets in radians
    delta_lat = d_north / R
    delta_lng = d_east / (R * cos(pi * old_lat / 180))

    # OffsetPosition, decimal degrees
    new_lat = emergent_object_lkp_lat + delta_lat * 180 / pi
    new_lng = emergent_object_lkp_lng + delta_lng * 180 / pi

    # debug
    print new_lat
    print new_lng
    print '---------------------------------'

    return new_lat, new_lng

def calcEmergent(emergent_object_lkp_lat, emergent_object_lkp_lng, search_altitude, search_radius_ground_distance):
    # lists to store multiple waypoints
    waypoints_lat = []
    waypoints_lng = []
    waypoints_alt = []

    # angles that define aircraft heading
    heading_angle_outward = radians(45)
    heading_angle_south = radians(270)
    heading_angle_west = radians(180)
    heading_angle_north = radians(90)
    # heading_angle_east = radians(0)

    # iterable dictionary that pairs heading direction with the aircraft angle that will send it that way
    heading_angle_list = [heading_angle_outward, heading_angle_south, heading_angle_west, heading_angle_north]

    # emergent object last known position (lkp) in decimal degrees - provided by interop
    # emergent_object_lkp_lat = 49
    # emergent_object_lkp_lng = -122

    old_lat = emergent_object_lkp_lat
    old_lng = emergent_object_lkp_lng

    # first loop (smaller)
    for angle in heading_angle_list:
        print 'Heading angle: {} (degrees: {})'.format(angle, degrees(angle))

        d_north = search_radius_ground_distance * sin(angle)
        d_east = search_radius_ground_distance * cos(angle)

        # find the next waypoint
        next_lat, next_lng = findNextLatLng(old_lat, old_lng, d_north, d_east, emergent_object_lkp_lat, emergent_object_lkp_lng)

        # save the results and update old latitude/longitude values
        waypoints_lat.append(next_lat)
        waypoints_lng.append(next_lng)
        waypoints_alt.append(search_altitude)

        old_lat = next_lat
        old_lng = next_lng

    # increase search radius
    # search_radius_ground_distance += 7

    # second loop (bigger)
    # for angle in heading_angle_list:
    #     print 'Heading angle: {} (degrees: {})'.format(angle, degrees(angle))
    #
    #     d_north = search_radius_ground_distance * sin(angle)
    #     d_east = search_radius_ground_distance * cos(angle)
    #
    #     # find the next waypoint
    #     next_lat, next_lng = findNextLatLng(old_lat, old_lng, d_north, d_east, emergent_object_lkp_lat, emergent_object_lkp_lng)
    #
    #     # save the results and update old latitude/longitude values
    #     waypoints_lat.append(next_lat)
    #     waypoints_lng.append(next_lng)
    #     waypoints_alt.append(search_altitude)
    #
    #     old_lat = next_lat
    #     old_lng = next_lng

    return {'lats': waypoints_lat, 'longs': waypoints_lng, 'alts': waypoints_alt}