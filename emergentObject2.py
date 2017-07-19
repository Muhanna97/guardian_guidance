from math import sin, cos, asin, atan2, radians as rad, degrees as deg
#


def calcEmergentSearch(latitude, longitude, searchAltitude, searchRadius, numSpirals):
    bearings = [315, 225, 135, 45]

    print "emergent location: ",latitude, longitude
    lats = []
    longs = []
    alts = []

    for j in range(numSpirals):
        for i in range(4):
            points = locationFromBearing(latitude, longitude, bearings[i], searchRadius*(j+1))
            print points
            lats.append(points['lat'])
            longs.append(points['long'])
            alts.append(searchAltitude)

        lats.append(lats[j*4]) # copy first wp to start
        longs.append(longs[j*4])
        alts.append(alts[j*4])

    print lats
    print longs

    return {'lats': lats,'longs':longs,'alts':alts}



def locationFromBearing(lat, long, bearing, distanceOut):

    #http://www.movable-type.co.uk/scripts/latlong.html#destPoint
    bearing = rad(bearing)

    R = 6378137.0

    latitude = asin(sin(rad(lat))*cos(distanceOut/R) + cos(rad(lat))*sin(distanceOut/R)*cos(bearing))
    longitude = rad(long) + atan2(sin(bearing)*sin(distanceOut/R)*cos(rad(lat)),cos(distanceOut/R)-sin(rad(lat))*sin(latitude))

    return {'lat': deg(latitude), 'long': deg(longitude)}